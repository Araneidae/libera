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

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "cordic.h"
#include "numeric.h"
#include "interlock.h"
#include "conditioning.h"
#include "waveform.h"

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

/* Selected attenuation.  The default is quite high for safety. */
static int CurrentAttenuation = 60;


/* Current scaling factor.  This is used to program the nominal beam current
 * for an input power of 0dBm, or equivalently, the beam current
 * corresponding to a button current of 4.5mA.
 *    This is recorded in units of 10nA, giving a maximum 0dBm current of
 * 20A. */
static int CurrentScale = 100000000;





/****************************************************************************/
/*                                                                          */
/*                             Switches and DSC                             */
/*                                                                          */
/****************************************************************************/


/* Forward declaration: CF:DSC and CF:AUTOSW affect each other directly. */
static void UpdateSc(int NewScState);


static void UpdateAutoSwitch(bool NewSwitchState)
{
    AutoSwitchState = NewSwitchState;
    SwitchReadback->Write(AutoSwitchState);
    
    if (!AutoSwitchState  &&  ScState == SC_MODE_AUTO)
        /* The switches cannot be switched away from automatic mode without
         * first turning signal conditioning off. */
        UpdateSc(SC_MODE_FIXED);

    WriteAutoSwitches(NewSwitchState);
}


static void UpdateSc(int NewScState)
{
    ScState = NewScState;
    ScReadback->Write(ScState);
    
    if (ScState == SC_MODE_AUTO)
        UpdateAutoSwitch(true);
    WriteScMode((SC_MODE) ScState);
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


/* This contains a precalculation of K_S * 10^((A-A_0)/20) to ensure that the
 * calculation of ComputeScaledCurrent is efficient. */
static PMFP ScaledCurrentFactor;
/* This contains a precalculation of 10^((A-A_0)/20): this only needs to
 * change when the attenuator settings are changed. */
static PMFP AttenuatorScalingFactor;



/* Returns the current cached attenuator setting. */

int ReadCorrectedAttenuation()
{
    return CurrentAttenuation * DB_SCALE;
}


static void UpdateCurrentScale()
{
    ScaledCurrentFactor = AttenuatorScalingFactor * CurrentScale;
}


/* Updates the attenuators and the associated current scaling factors. */

void UpdateAttenuation(int NewAttenuation)
{
    ScWriteAttenuation(NewAttenuation);
    CurrentAttenuation = NewAttenuation;
    /* Update the scaling factors. */
    AttenuatorScalingFactor = PMFP(from_dB, ReadCorrectedAttenuation() - A_0);
    UpdateCurrentScale();
}



/* Converts a raw current (or charge) intensity value into a scaled current
 * value. */

int ComputeScaledCurrent(const PMFP & IntensityScale, int Intensity)
{
    return Denormalise(IntensityScale * ScaledCurrentFactor * Intensity);
}



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

    /* Note that updating the attenuators is done via the
     * InterlockedUpdateAttenuation() routine: this will not actually update
     * the attenuators (by calling UpdateAttenuation() above) until the
     * interlock mechanism is ready. */
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN",
        CurrentAttenuation, UpdateAttenuation);
    
    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    UpdateAttenuation(CurrentAttenuation);
    UpdateAutoSwitch(AutoSwitchState);
    UpdateManualSwitch();
    UpdateSc(ScState);
    UpdateSwitchTrigger();
    UpdateSwitchTriggerDelay();

    return true;
}
