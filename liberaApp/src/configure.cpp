/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#include <dbFldTypes.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
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
/*                          Spike Removal Control                           */
/*                                                                          */
/****************************************************************************/


static bool EnableSpikeRemoval = true;
static int SpikeAverageWindow = 3;
static int SpikeAverageStop = -1;
static int SpikeStart = -3;
static int SpikeWindow = 8;


class SPIKE_DEBUG : public I_WAVEFORM
{
public:
    SPIKE_DEBUG(const char * Name) :
        I_WAVEFORM(DBF_LONG)
    {
        Publish_waveform(Name, *this);
    }

    bool process(
        void *array, size_t max_length, size_t &new_length)
    {
        if (max_length >= SPIKE_DEBUG_BUFLEN)
        {
            int * Buffer = (int *) array;
            new_length = SPIKE_DEBUG_BUFLEN;
            return ReadSpikeRemovalBuffer(Buffer);
        }
        else
            return false;
    }
};


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
    PUBLISH_SPIKE(bo,      "ENABLE",    EnableSpikeRemoval);
    PUBLISH_SPIKE(mbbo,    "AVEWIN",    SpikeAverageWindow);
    PUBLISH_SPIKE(longout, "AVESTOP",   SpikeAverageStop);
    PUBLISH_SPIKE(longout, "SPIKEST",   SpikeStart);
    PUBLISH_SPIKE(longout, "SPIKEWIN",  SpikeWindow);

    new SPIKE_DEBUG("CF:SR:DEBUGWF");
    
    UpdateSpikeRemoval();
    return true;
}
#undef PUBLISH_SPIKE



/****************************************************************************/
/*                                                                          */
/*                           Notch Filter Control                           */
/*                                                                          */
/****************************************************************************/


static bool NotchFilterEnabled = true;

static NOTCH_FILTER DisabledNotchFilters[2];
static NOTCH_FILTER NotchFilters[2];

#define NOTCH_FILTER_FILENAME   "/opt/lib/notch%d"


static void SetNotchFilterEnable()
{
    if (NotchFilterEnabled)
    {
        WriteNotchFilter(0, NotchFilters[0]);
        WriteNotchFilter(1, NotchFilters[1]);
    }
    else
    {
        WriteNotchFilter(0, DisabledNotchFilters[0]);
        WriteNotchFilter(1, DisabledNotchFilters[1]);
    }
}


static bool InitialiseNotchFilterEnable()
{
    bool Ok = true;
    for (int filter_index = 0; Ok && filter_index < 2; filter_index++)
    {
        int *Filter = NotchFilters[filter_index];
        char FilterFileName[80];
        sprintf(FilterFileName, NOTCH_FILTER_FILENAME, filter_index + 1);

        /* Read the entire file in one go.  Saves hassle fighting with
         * stupidities of fscanf(), though this code is much more
         * complicated! */
        int Input;
        char NotchFile[1024 + 1];
        int Read = 0;
        Ok =
            TEST_IO(Input = open(FilterFileName, O_RDONLY))  &&
            TEST_IO(Read  = read(Input, NotchFile, 1024));
        if (Ok)
        {
            NotchFile[Read] = '\0';    // Ensure file is null terminated
            char * String = NotchFile;
            for (int i = 0; Ok && i < 5; i ++)
            {
                char * End;
                Filter[i] = strtoul(String, &End, 0);
                errno = 0;
                Ok = TEST_OK(End > String);
                String = End;
            }
        }
        if (Input != -1)
            close(Input);

        if (Ok)
        {
            /* After successfully loading NotchFilters[filter_index] compute
             * the corresponding disabled filter.  This should have the same
             * DC response as the original filter, computed as
             *
             *      2^17 * sum(numerator) / sum(denominator)
             *
             * where numerator is coefficients 0,1,2 and denominator is
             * coefficients 1,2 together with a constant factor of 2^17. */
            int numerator = Filter[0] + Filter[1] + Filter[2];
            int denominator = 0x20000 + Filter[3] + Filter[4];
            int *Disabled = DisabledNotchFilters[filter_index];
            int response = (int) ((0x20000LL * numerator) / denominator);
            if (response >= 0x20000)
                response = 0x1FFFF;
            memset(Disabled, 0, sizeof(NOTCH_FILTER));
            Disabled[0] = response;
        }
    }
    return Ok;
}


/****************************************************************************/
/*                                                                          */

static void SetBpmEnabled()
{
    /* At the moment the only things affected by the ENABLED flag are the
     * overall system health (managed in the EPICS database) and the
     * interlock. */
    NotifyInterlockBpmEnable(BpmEnabled);
}


bool InitialiseConfigure()
{
    /* Enable the configuration features that need special initialisation. */
    bool Ok =
        InitialiseNotchFilterEnable()  &&
        IF_(Version2FpgaPresent, InitialiseSpikeRemoval());
    if (!Ok)
        return false;
    
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
    PUBLISH_CONFIGURATION(longout, "CF:TRIGDLY",
        ExternalTriggerDelay, WriteExternalTriggerDelay);

    PUBLISH_FUNCTION_OUT(bo, "CF:NOTCHEN",
        NotchFilterEnabled, SetNotchFilterEnable);
    
    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    UpdateAutoSwitch(AutoSwitchState);
    UpdateManualSwitch();
    UpdateSc(ScState);
    UpdateSwitchTrigger();
    UpdateSwitchTriggerDelay();
    WriteExternalTriggerDelay(ExternalTriggerDelay);
    SetNotchFilterEnable();

    return true;
}
