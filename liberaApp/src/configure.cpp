/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
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
/*                              Filter Control                              */
/*                                                                          */
/****************************************************************************/


class DECIMATION_FILTER : public I_WAVEFORM
{
public:
    DECIMATION_FILTER(
        const char *name, const char *filename, int wf_length,
        void (*on_update)(const int *)):

        I_WAVEFORM(DBF_LONG),
        filename(filename),
        wf_length(wf_length),
        on_update(on_update),
        filter(new int[wf_length])
    {
        ok = load_file();
        do_reload = false;
        enabled = true;
        Publish_waveform(name, *this);
    }

    /* Resets filter back to filter loaded from file and ensures that next EPICS
     * process event will run backwards so the EPICS interface sees the reloaded
     * filter. */
    void Reset(void)
    {
        ok = load_file();
        update_filter();
        do_reload = true;
    }

    /* Updates the enabled state.  Only safe to call if GetDisabled() has been
     * overridden to return a non-null value. */
    void SetEnabled(bool Enabled)
    {
        enabled = Enabled;
        update_filter();
    }


protected:
    /* Returns the waveform to use when disabling, or NULL if not supported. */
    virtual int *GetDisabled(void) { return NULL; }


private:
    bool load_file(void)
    {
        FILE *input = fopen(filename, "r");
        bool ok = input != NULL;
        if (!ok)
            printf("Unable to open filter file \"%s\"\n", filename);
        else
        {
            for (size_t i = 0; ok  &&  i < wf_length; i ++)
                ok = TEST_OK(fscanf(input, "%i\n", &filter[i]) == 1);
            fclose(input);
        }
        /* If loading failed reset the filter to zero. */
        if (!ok)
            memset(filter, 0, sizeof(int) * wf_length);
        return ok;
    }

    /* Called each time the filter has changed. */
    void update_filter(void)
    {
        if (ok)
            on_update(enabled ? filter : GetDisabled());
    }

    bool process(void *array, size_t max_length, size_t &new_length)
    {
        size_t length = max_length < wf_length ? max_length : wf_length;
        new_length = length;
        if (do_reload)
        {
            /* On reload we force a process where we write our state back to
             * EPICS. */
            memcpy(array, filter, sizeof(int) * length);
            do_reload = false;
        }
        else
        {
            /* On normal processing we read from EPICS and update the underlying
             * filter.  Any points not assigned are set to zero. */
            memcpy(filter, array, sizeof(int) * length);
            if (length < wf_length)
                memset(filter + length, 0, sizeof(int) * (wf_length - length));
            ok = true;
            if (enabled)
                update_filter();
        }
        return ok;
    }

    bool init(void *array, size_t &length)
    {
        length = wf_length;
        memcpy(array, filter, wf_length * sizeof(int));
        return ok;
    }


    const char *const filename;
    const size_t wf_length;
    void (*const on_update)(const int *);

    bool ok;
    bool enabled;
    bool do_reload;

protected:
    int *const filter;
};


class NOTCH_FILTER : public DECIMATION_FILTER
{
public:
    NOTCH_FILTER(
        const char *name, const char *filename,
        void (*on_update)(const int *)):

        DECIMATION_FILTER(name, filename, 5, on_update)
    {
        /* After successfully loading NotchFilters[filter_index] compute the
         * corresponding disabled filter.  This should have the same DC response
         * as the original filter, computed as
         *
         *      2^17 * sum(numerator) / sum(denominator)
         *
         * where numerator is coefficients 0,1,2 and denominator is coefficients
         * 1,2 together with a constant factor of 2^17. */
        int numerator = filter[0] + filter[1] + filter[2];
        int denominator = 0x20000 + filter[3] + filter[4];
        int response = (int) ((0x20000LL * numerator) / denominator);
        if (response >= 0x20000)
            response = 0x1FFFF;
        memset(disabled_filter, 0, sizeof(disabled_filter));
        disabled_filter[0] = response;
    }


private:
    int *GetDisabled(void) { return disabled_filter; }

    /* Disabled version of notch filter with same response as original
     * version. */
    int disabled_filter[5];
};


class FILTER_CONTROL
{
public:
    FILTER_CONTROL(void) :
        notch_1("CF:NOTCH1", "/opt/lib/notch1", WriteNotchFilter1),
        notch_2("CF:NOTCH2", "/opt/lib/notch2", WriteNotchFilter2),
        fir("CF:FIR", "/opt/lib/polyphase_fir",
            FA_DecimationFirLength, WriteFA_FIR)
    {
        NotchFilterEnabled = true;
        PUBLISH_METHOD_OUT(bo, "CF:NOTCHEN",
            SetNotchFilterEnable, NotchFilterEnabled);
        PUBLISH_METHOD_ACTION("CF:RESETFA", ResetFilters);

        notch_1.SetEnabled(true);
        notch_2.SetEnabled(true);
        fir.SetEnabled(true);
    }

private:
    bool SetNotchFilterEnable(bool Enabled)
    {
        notch_1.SetEnabled(Enabled);
        notch_2.SetEnabled(Enabled);
        return true;
    }

    bool ResetFilters(void)
    {
        notch_1.Reset();
        notch_2.Reset();
        fir.Reset();
        return true;
    }

    NOTCH_FILTER notch_1, notch_2;
    DECIMATION_FILTER fir;

    bool NotchFilterEnabled;
};


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

    new FILTER_CONTROL;

    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    UpdateAutoSwitch(AutoSwitchState);
    UpdateManualSwitch();
    UpdateSc(ScState);
    UpdateSwitchTrigger();
    UpdateSwitchTriggerDelay();
    WriteExternalTriggerDelay(ExternalTriggerDelay);

    return true;
}
