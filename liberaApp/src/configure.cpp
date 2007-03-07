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

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "cordic.h"
#include "numeric.h"
#include "interlock.h"

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
READBACK<bool> * SwitchReadback = NULL;
/* Selects which switch setting to use in manual mode. */
static int ManualSwitch = 3;


/* Selects internal or external triggering for the rotating switches. */
static bool ExternalSwitchTrigger = false;
/* Selects the delay from external trigger for the switches. */
static int SwitchTriggerDelay = 0;

/* Controls the Digital Signal Conditioning daemon state. */
static int DscState = 0;
READBACK<int> * DscReadback = NULL;

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


static void UpdateDsc(int NewDscState);


/* Called whenever the autoswitch mode has changed. */

static void UpdateManualSwitch()
{
    /* Only update the switches if they're in manual mode. */
    if (!AutoSwitchState)
        WriteSwitchState((CSPI_SWITCHMODE) ManualSwitch);
}


static void UpdateAutoSwitch(bool NewSwitchState)
{
    AutoSwitchState = NewSwitchState;
    SwitchReadback->Write(AutoSwitchState);
    
    if (AutoSwitchState)
        WriteSwitchState(CSPI_SWITCH_AUTO);
    else
    {
        if (DscState == CSPI_DSC_AUTO)
            UpdateDsc(CSPI_DSC_OFF);
        UpdateManualSwitch();
    }
}


int ReadSwitchSetting()
{
    return ManualSwitch;
}


#define REGISTER_SWITCH_TRIGGER         0x1400C038
#define REGISTER_SWITCH_DELAY           0x1400C03C

static void UpdateSwitchTrigger()
{
    /* Read the current trigger setting (to get the interval count) and
     * update the top bit as required to control the trigger source. */
    unsigned int SwitchControl;
    if (ReadRawRegister(REGISTER_SWITCH_TRIGGER, SwitchControl))
    {
        SwitchControl =
            (SwitchControl & 0x7FFFFFFF) | (ExternalSwitchTrigger << 31);
        WriteRawRegister(REGISTER_SWITCH_TRIGGER, SwitchControl);
    }
}

static void UpdateSwitchTriggerDelay()
{
    WriteRawRegister(REGISTER_SWITCH_DELAY, SwitchTriggerDelay);
}


static void UpdateDsc(int NewDscState)
{
    DscState = NewDscState;
    DscReadback->Write(DscState);
    
    if (DscState == CSPI_DSC_AUTO)
        UpdateAutoSwitch(true);
    WriteDscMode((CSPI_DSCMODE) DscState);
}


static void WriteDscStateFile()
{
    WriteDscMode(CSPI_DSC_SAVE_LASTGOOD);
}



/****************************************************************************/
/*                                                                          */
/*                          Attenuation Management                          */
/*                                                                          */
/****************************************************************************/



/* Attenuator configuration management. */


#define MAX_ATTENUATION  62
#define OFFSET_CONF_FILE "/opt/dsc/offsets.conf"


/* The attenuator value reported by ReadCachedAttenuation() is not strictly
 * accurate, due to minor offsets on attenuator values.  Here we attempt to
 * compensate for these offsets by reading an offset configuration file. */
static int AttenuatorOffset[MAX_ATTENUATION + 1];


/* This contains a precalculation of K_S * 10^((A-A_0)/20) to ensure that the
 * calculation of ComputeScaledCurrent is efficient. */
static PMFP ScaledCurrentFactor;
/* This contains a precalculation of 10^((A-A_0)/20): this only needs to
 * change when the attenuator settings are changed. */
static PMFP AttenuatorScalingFactor;


static bool ReadAttenuatorOffsets()
{
    FILE * OffsetFile = fopen(OFFSET_CONF_FILE, "r");
    if (OffsetFile == NULL)
    {
        printf("Unable to open file " OFFSET_CONF_FILE "\n");
        return false;
    }
    else
    {
        bool Ok = true;
        for (int i = 0; Ok  &&  i <= MAX_ATTENUATION; i ++)
        {
            double Offset;
            Ok = fscanf(OffsetFile, "%lf", &Offset) == 1;
            if (Ok)
                AttenuatorOffset[i] = (int) (DB_SCALE * Offset);
            else
                printf("Error reading file " OFFSET_CONF_FILE "\n");
        }
        fclose(OffsetFile);
        return Ok;
    }
}


/* Returns the current cached attenuator setting, after correcting for
 * attenuator offset. */

int ReadCorrectedAttenuation()
{
    return CurrentAttenuation * DB_SCALE +
        AttenuatorOffset[CurrentAttenuation];
}


static void UpdateCurrentScale()
{
    ScaledCurrentFactor = AttenuatorScalingFactor * CurrentScale;
}


/* Updates the attenuators and the associated current scaling factors. */

void UpdateAttenuation(int NewAttenuation)
{
    CurrentAttenuation = NewAttenuation;
    WriteAttenuation(CurrentAttenuation);
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
    if (!ReadAttenuatorOffsets())
        return false;

    /* Master enable flag.  Disabling this has little practical effect on BPM
     * outputs (apart from disabling interlock), but is available as a global
     * PV for BPM management. */
    PUBLISH_CONFIGURATION(bo, "CF:ENABLED", BpmEnabled, SetBpmEnabled);
        

    SwitchReadback = PUBLISH_READBACK_CONFIGURATION(bi, bo, "CF:AUTOSW",
        AutoSwitchState, UpdateAutoSwitch);
    PUBLISH_CONFIGURATION(longout, "CF:SETSW", 
        ManualSwitch, UpdateManualSwitch);
    PUBLISH_CONFIGURATION(bo, "CF:TRIGSW", 
        ExternalSwitchTrigger, UpdateSwitchTrigger);
    PUBLISH_CONFIGURATION(longout, "CF:DELAYSW", 
        SwitchTriggerDelay, UpdateSwitchTriggerDelay);
    DscReadback = PUBLISH_READBACK_CONFIGURATION(mbbi, mbbo, "CF:DSC",
        DscState, UpdateDsc);
    PUBLISH_CONFIGURATION(ao, "CF:ISCALE", CurrentScale, UpdateCurrentScale);
    PUBLISH_ACTION("CF:WRITEDSC", WriteDscStateFile);

    /* Note that updating the attenuators is done via the
     * InterlockedUpdateAttenuation() routine: this will not actually update
     * the attenuators (by calling UpdateAttenuation() above) until the
     * interlock mechanism is ready. */
    PUBLISH_CONFIGURATION(longout, "CF:ATTEN", 
        CurrentAttenuation, InterlockedUpdateAttenuation);
    
    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    WriteAgcMode(CSPI_AGC_MANUAL);
    InterlockedUpdateAttenuation(CurrentAttenuation);
    UpdateAutoSwitch(AutoSwitchState);
    UpdateDsc(DscState);
    UpdateSwitchTrigger();
    UpdateSwitchTriggerDelay();

    return true;
}
