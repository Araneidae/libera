/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009  Michael Abbott, Diamond Light Source Ltd.
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

/* Attenuator configuration management. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <dbFldTypes.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "numeric.h"
#include "conditioning.h"
#include "waveform.h"

#include "attenuation.h"



/* Current scaling factor.  This is used to program the nominal beam current
 * for an input power of 0dBm, or equivalently, the beam current
 * corresponding to a button current of 4.5mA.
 *    This is recorded in units of 10nA, giving a maximum 0dBm current of
 * 20A. */
static int CurrentScale = 100000000;


/* AGC control. */
static bool AgcEnable = false;
static int AgcUpThreshold = 70;
static int AgcDownThreshold = 20;

static READBACK<int> *AttenReadback = NULL;


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
 * is called each time any of the attenuation settings changes.  Returns
 * false if nothing actually changed. */

static bool UpdateAttenuation(bool ForceUpdate)
{
    int MaxAttenuation = MaximumAttenuation();
    int NewAttenuation = SelectedAttenuation + AttenuatorDelta;
    if (NewAttenuation < 0)
        NewAttenuation = 0;
    if (NewAttenuation > MaxAttenuation)
        NewAttenuation = MaxAttenuation;

    if (CurrentAttenuation == NewAttenuation  &&  !ForceUpdate)
        return false;
    else
    {
        CurrentAttenuation = NewAttenuation;
        CorrectedAttenuation = CurrentAttenuation * DB_SCALE +
            AttenuatorOffsets[CurrentAttenuation];
        ScWriteAttenuation(CurrentAttenuation);

        /* Update the scaling factors. */
        AttenuatorScalingFactor = PMFP(from_dB, CorrectedAttenuation - A_0);
        UpdateCurrentScale();

        return true;
    }
}

/* Called from EPICS when the attenuation offset or delta has changed. */

static void DoUpdateAttenuation()
{
    UpdateAttenuation(false);
}


/* Called from EPICS when the selected attenuation is changed. */

static bool SelectNewAttenuation(int NewAttenuation)
{
    SelectedAttenuation = NewAttenuation;
    DoUpdateAttenuation();
    return true;
}


void NotifyMaxAdc(int MaxAdc)
{
    if (AgcEnable)
    {
        int Percent = 100 * MaxAdc / 32768;
        int NewAttenuation = SelectedAttenuation;
        if (Percent >= AgcUpThreshold)
            NewAttenuation += 1;
        else if (Percent <= AgcDownThreshold)
            NewAttenuation -= 1;

        /* Ensure the new attenuation doesn't go outside the selectable
         * bounds. */
        if (NewAttenuation < 0)   NewAttenuation = 0;
        if (NewAttenuation > 62)  NewAttenuation = 62;
        if (NewAttenuation != SelectedAttenuation)
        {
            SelectNewAttenuation(NewAttenuation);
            AttenReadback->Write(NewAttenuation);
        }
    }
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





bool InitialiseAttenuation()
{
    PUBLISH_CONFIGURATION(ao, "CF:ISCALE", CurrentScale, UpdateCurrentScale);
    
    AttenReadback = PUBLISH_READBACK_CONFIGURATION(
        longin, longout, "CF:ATTEN",
        SelectedAttenuation, SelectNewAttenuation);
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN:DISP",
        AttenuatorDelta, DoUpdateAttenuation);
    new ATTENUATOR_OFFSETS("CF:ATTEN:OFFSET", DoUpdateAttenuation);
    Publish_ai("CF:ATTEN:TRUE", CorrectedAttenuation);

    PUBLISH_CONFIGURATION(bo, "CF:ATTEN:AGC", AgcEnable, NULL_ACTION);
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN:AGC:DN",
        AgcDownThreshold, NULL_ACTION);
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN:AGC:UP",
        AgcUpThreshold, NULL_ACTION);

    UpdateAttenuation(true);
    return true;
}

