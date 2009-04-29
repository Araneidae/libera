/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2007  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */


/* Libera position calculations and conversions. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <dbFldTypes.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "cordic.h"
#include "numeric.h"
#include "interlock.h"
#include "conditioning.h"
#include "waveform.h"
#include "versions.h"

#include "configure.h"



/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/

/* Master enable flag. */
static bool BpmEnabled = true;


/* Control configuration. */

/* Controls the rotating switches: manual or automatic mode. */
static bool AutoSwitchState = false;
static READBACK<bool> * SwitchReadback = NULL;
/* Selects which switch setting to use in manual mode. */
static int ManualSwitch = 3;
/* The permutation corresponding to the selected switch position is published
 * for the use of external procedures. */
static INT_WAVEFORM Permutation(4);


/* Selects internal or external triggering for the rotating switches. */
static bool ExternalSwitchTrigger = false;
/* Selects the delay from external trigger for the switches. */
static int SwitchTriggerDelay = 0;

/* Controls the Signal Conditioning process state. */
static int ScState = SC_MODE_FIXED;
static READBACK<int> * ScReadback = NULL;

/* Current scaling factor.  This is used to program the nominal beam current
 * for an input power of 0dBm, or equivalently, the beam current
 * corresponding to a button current of 4.5mA.
 *    This is recorded in units of 10nA, giving a maximum 0dBm current of
 * 20A. */
static int CurrentScale = 100000000;

/* Delay from external trigger in clocks. */
static int ExternalTriggerDelay = 0;




/****************************************************************************/
/*                                                                          */
/*                             Switches and DSC                             */
/*                                                                          */
/****************************************************************************/


/* Forward declaration: CF:DSC and CF:AUTOSW affect each other directly. */
static bool UpdateSc(int NewScState);


static bool UpdateAutoSwitch(bool NewSwitchState)
{
    AutoSwitchState = NewSwitchState;
    SwitchReadback->Write(AutoSwitchState);
    
    if (!AutoSwitchState  &&  ScState == SC_MODE_AUTO)
        /* The switches cannot be switched away from automatic mode without
         * first turning signal conditioning off. */
        UpdateSc(SC_MODE_FIXED);

    WriteAutoSwitches(NewSwitchState);
    return true;
}


static bool UpdateSc(int NewScState)
{
    ScState = NewScState;
    ScReadback->Write(ScState);
    
    if (ScState == SC_MODE_AUTO)
        UpdateAutoSwitch(true);
    WriteScMode((SC_MODE) ScState);
    return true;
}


/* Called whenever the autoswitch mode has changed. */

static void UpdateManualSwitch()
{
    /* Only update the switches if they're in manual mode. */
    WriteManualSwitches(ManualSwitch);
    /* Update the permutation. */
    memcpy(Permutation.Array(), SwitchPermutation(), sizeof(PERMUTATION));
}


static void UpdateSwitchTrigger()
{
    WriteSwitchTriggerSelect(ExternalSwitchTrigger);
}

static void UpdateSwitchTriggerDelay()
{
    WriteSwitchTriggerDelay(SwitchTriggerDelay);
}




/****************************************************************************/
/*                                                                          */
/*                          Attenuation Management                          */
/*                                                                          */
/****************************************************************************/

/* Attenuator configuration management. */


/* This is the attenuation as selected by the operator.  This needs to be
 * adjusted by Delta and Offsets[Delta]. */
static int SelectedAttenuation = 60;
/* To help with aligning attenuator settings among multiple Liberas, and
 * particular to help with interoperation with Brilliance we allow the
 * attenuator setting to be adjusted by the delta factor. */
static int AttenuatorDelta;
/* Selected attenuation.  The default is quite high for safety.  This is the
 * true attenuation after correction by AttenuatorDelta (and clipping), but
 * not corrected for offset. */
static int CurrentAttenuation = 60;


/* The attenuator value reported by ReadCachedAttenuation() is not strictly
 * accurate, due to minor offsets on attenuator values.  Here we attempt to
 * compensate for these offsets by reading an offset configuration file. */
static int * AttenuatorOffsets;
/* This is the corrected attenuation. */
static int CorrectedAttenuation;

/* This contains a precalculation of K_S * 10^((A-A_0)/20) to ensure that the
 * calculation of ComputeScaledCurrent is efficient. */
static PMFP ScaledCurrentFactor;
/* This contains a precalculation of 10^((A-A_0)/20): this only needs to
 * change when the attenuator settings are changed. */
static PMFP AttenuatorScalingFactor;


class ATTENUATOR_OFFSETS : public I_WAVEFORM
{
public:
    ATTENUATOR_OFFSETS(const char *Name, void (*on_update)()) :
        I_WAVEFORM(DBF_FLOAT),
        AttenuatorCount(MaximumAttenuation() + 1),
        on_update(on_update)
    {
        AttenuatorOffsets = (int *) calloc(AttenuatorCount, sizeof(int));
        memset(AttenuatorOffsets, 0, AttenuatorCount * sizeof(int));

        Publish_waveform(Name, *this);
        PersistentWaveform(Name, AttenuatorOffsets, AttenuatorCount);
    }

    bool process(
        void *array, size_t max_length, size_t &new_length)
    {
        float * farray = (float *) array;
        size_t i;
        for (i = 0; i < new_length; i ++)
            AttenuatorOffsets[i] = (int) (DB_SCALE * farray[i]);
        /* In case only part of the waveform was assigned (new_length <
         * max_length) restore the rest of the array from the stored array.
         * Otherwise AttenuatorOffsets and ATTEN:OFFSET_S will fall out of
         * step. */
        for (; i < max_length; i ++)
            farray[i] = (float) AttenuatorOffsets[i] / DB_SCALE;
        new_length = max_length;
        
        PERSISTENT_BASE::MarkDirty();
        on_update();
        return true;
    }

    bool init(void *array, size_t &length)
    {
        float * farray = (float *) array;
        for (size_t i = 0; i < AttenuatorCount; i ++)
            farray[i] = (float) AttenuatorOffsets[i] / DB_SCALE;
        length = AttenuatorCount;
        return true;
    }
    
private:
    const size_t AttenuatorCount;
    void (*on_update)();
};


static void UpdateCurrentScale()
{
    ScaledCurrentFactor = AttenuatorScalingFactor * CurrentScale;
}


/* Updates the attenuators and the associated current scaling factors.  This
 * is called each time any of the attenuation settings changes. */

static void UpdateAttenuation()
{
    int MaxAttenuation = MaximumAttenuation();
    int NewAttenuation = SelectedAttenuation + AttenuatorDelta;
    if (NewAttenuation < 0)
        NewAttenuation = 0;
    if (NewAttenuation > MaxAttenuation)
        NewAttenuation = MaxAttenuation;

    CurrentAttenuation = NewAttenuation;
    CorrectedAttenuation = CurrentAttenuation * DB_SCALE +
        AttenuatorOffsets[CurrentAttenuation];
    ScWriteAttenuation(CurrentAttenuation);
    
    /* Update the scaling factors. */
    AttenuatorScalingFactor = PMFP(from_dB, CorrectedAttenuation - A_0);
    UpdateCurrentScale();
}



/* Returns the current cached attenuator setting.  This is scaled by DB_SCALE
 * and represents an estimate of the true attenuator setting. */

int ReadCorrectedAttenuation()
{
    return CorrectedAttenuation;
}


/* Converts a raw current (or charge) intensity value into a scaled current
 * value. */

int ComputeScaledCurrent(const PMFP & IntensityScale, int Intensity)
{
    return Denormalise(IntensityScale * ScaledCurrentFactor * Intensity);
}



/****************************************************************************/
/*                                                                          */
/*                          Spike Removal Control                           */
/*                                                                          */
/****************************************************************************/


static bool EnableSpikeRemoval = true;
static int SpikeAverageWindow = 8;
static int SpikeAverageStop = -1;
static int SpikeStart = -3;
static int SpikeWindow = 8;


static void UpdateSpikeRemoval()
{
    WriteSpikeRemovalSettings(
        EnableSpikeRemoval,
        SpikeAverageWindow, SpikeAverageStop, SpikeStart, SpikeWindow);
}


#define PUBLISH_SPIKE(record, name, variable) \
    PUBLISH_CONFIGURATION(record, "CF:SR:" name, variable, UpdateSpikeRemoval)
static bool InitialiseSpikeRemoval()
{
    PUBLISH_SPIKE(bo, "ENABLE", EnableSpikeRemoval);
    PUBLISH_SPIKE(longout, "AVEWIN",    SpikeAverageWindow);
    PUBLISH_SPIKE(longout, "AVESTOP",   SpikeAverageStop);
    PUBLISH_SPIKE(longout, "SPIKEST",   SpikeStart);
    PUBLISH_SPIKE(longout, "SPIKEWIN",  SpikeWindow);
    
    UpdateSpikeRemoval();
    return true;
}
#undef PUBLISH_SPIKE


/****************************************************************************/
/*                                                                          */

void SetBpmEnabled()
{
    /* At the moment the only things affected by the ENABLED flag are the
     * overall system health (managed in the EPICS database) and the
     * interlock. */
    NotifyInterlockBpmEnable(BpmEnabled);
}


bool InitialiseConfigure()
{
    /* Master enable flag.  Disabling this has little practical effect on BPM
     * outputs (apart from disabling interlock), but is available as a global
     * PV for BPM management. */
    PUBLISH_CONFIGURATION(bo, "CF:ENABLED", BpmEnabled, SetBpmEnabled);

    SwitchReadback = PUBLISH_READBACK_CONFIGURATION(bi, bo, "CF:AUTOSW",
        AutoSwitchState, UpdateAutoSwitch);
    PUBLISH_CONFIGURATION(longout, "CF:SETSW", 
        ManualSwitch, UpdateManualSwitch);
    Publish_waveform("CF:PERM", Permutation);
    PUBLISH_CONFIGURATION(bo, "CF:TRIGSW", 
        ExternalSwitchTrigger, UpdateSwitchTrigger);
    PUBLISH_CONFIGURATION(longout, "CF:DELAYSW", 
        SwitchTriggerDelay, UpdateSwitchTriggerDelay);
    ScReadback = PUBLISH_READBACK_CONFIGURATION(mbbi, mbbo, "CF:DSC",
        ScState, UpdateSc);
    PUBLISH_CONFIGURATION(ao, "CF:ISCALE", CurrentScale, UpdateCurrentScale);
    PUBLISH_CONFIGURATION(longout, "CF:TRIGDLY",
        ExternalTriggerDelay, WriteExternalTriggerDelay);

    /* Note that updating the attenuators is done via the
     * InterlockedUpdateAttenuation() routine: this will not actually update
     * the attenuators (by calling UpdateAttenuation() above) until the
     * interlock mechanism is ready. */
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN",
        SelectedAttenuation, UpdateAttenuation);
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN:DISP",
        AttenuatorDelta, UpdateAttenuation);
    new ATTENUATOR_OFFSETS("CF:ATTEN:OFFSET", UpdateAttenuation);
    Publish_ai("CF:ATTEN:TRUE", CorrectedAttenuation);
    
    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    UpdateAttenuation();
    UpdateAutoSwitch(AutoSwitchState);
    UpdateManualSwitch();
    UpdateSc(ScState);
    UpdateSwitchTrigger();
    UpdateSwitchTriggerDelay();
    WriteExternalTriggerDelay(ExternalTriggerDelay);

    /* Finally enable the 2.0 specific configuration features. */
    if (Version2FpgaPresent)
        return InitialiseSpikeRemoval();
    else
        return true;
}
