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


/* Provide support for free running "turn by turn" data.  This data is
 * acquired continously. */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"
#include "numeric.h"
#include "cordic.h"
#include "statistics.h"

#include "freeRun.h"



class FREE_RUN : I_EVENT, LOCKED
{
public:
    FREE_RUN(int WaveformLength) :
        WaveformLength(WaveformLength),
        WaveformIq(WaveformLength),
        InputAbcd(WaveformLength),
        WaveformAbcd(WaveformLength, true),
        WaveformXyqs(WaveformLength),
        PublishCapturedSamples(0),
        StatsX(WaveformXyqs, FIELD_X, "X"),
        StatsY(WaveformXyqs, FIELD_Y, "Y"),
        TuneX(WaveformXyqs, FIELD_X, "X"),
        TuneY(WaveformXyqs, FIELD_Y, "Y")
    {
        CaptureOffset = 0;
        AverageBits = 0;
        CapturedSamples = 0;
        StopWhenDone = false;
        UpdateAll = false;
        GotEpicsLock = false;
        
        /* Publish all the waveforms and the interlock. */
        WaveformIq.Publish("FR");
        WaveformAbcd.Publish("FR");
        WaveformXyqs.Publish("FR");
        Publish_longout("FR:DELAY", CaptureOffset);

        /* Averaging control. */
        Publish_longin("FR:SAMPLES", PublishCapturedSamples);
        PUBLISH_METHOD_OUT(longout, "FR:AVERAGE", SetAverageBits, AverageBits);
        Publish_bo("FR:AUTOSTOP", StopWhenDone);
        Publish_bo("FR:ALLUPDATE", UpdateAll);
        PUBLISH_METHOD_ACTION("FR:RESET", ResetAverage);

        Interlock.Publish("FR", true);
        Enable.Publish("FR");
        
        /* Announce our interest in the trigger. */
        RegisterTriggerEvent(*this, PRIORITY_FR);
    }


private:
    FREE_RUN();

    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /* Event handling.                                                       */
    
    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent(int)
    {
        /* Ignore events if not enabled. */
        if (!Enable.Enabled())
            return;
        /* If we've captured a full set of samples and we're configured to
         * stop then stop. */
        if (StopWhenDone  &&  CheckComplete())
            return;

        /* Before we do anything that might affect the variables we share
         * with EPICS ensure that EPICS isn't reading them.   In this case,
         * because we don't always trigger an update, we can end up capturing
         * the EPICS interlock without releasing it -- as this interlock only
         * affects FR processing, this is the behaviour we want. */
        if (!GotEpicsLock)
            Interlock.Wait();
        
        WaveformIq.Capture(1, CaptureOffset);
        if (AccumulateWaveform())
        {
            WaveformXyqs.CaptureConvert(WaveformAbcd);
            
            /* Compute our analysis on the X and Y waveforms, both position
             * statistics and tune response measurement. */
            StatsX.Update();
            TuneX.Update();
            StatsY.Update();
            TuneY.Update();

            /* Let EPICS know there's stuff to read, releases interlock. */
            Interlock.Ready(WaveformIq.GetTimestamp());
            GotEpicsLock = false;
        }
        else
            GotEpicsLock = true;
        
        PublishCapturedSamples.Write(CapturedSamples);
    }


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /* Waveform averaging.                                                   */
    
    bool SetAverageBits(int _AverageBits)
    {
        THREAD_LOCK(this);
        AverageBits = _AverageBits;
        ResetAccumulator();
        THREAD_UNLOCK();
        return true;
    }

    bool ResetAverage()
    {
        THREAD_LOCK(this);
        ResetAccumulator();
        THREAD_UNLOCK();
        return true;
    }

    void ResetAccumulator()
    {
        CapturedSamples = 0;
        memset(WaveformAbcd.Waveform(), 0, sizeof(ABCD_ROW) * WaveformLength);
    }

    void AccumulateAbcd()
    {
        CapturedSamples += 1;
        ABCD_ROW * Input = InputAbcd.Waveform();
        ABCD_ROW * Accum = WaveformAbcd.Waveform();
        for (int i = 0; i < WaveformLength; i ++)
        {
            Accum[i].A += Input[i].A >> AverageBits;
            Accum[i].B += Input[i].B >> AverageBits;
            Accum[i].C += Input[i].C >> AverageBits;
            Accum[i].D += Input[i].D >> AverageBits;
        }
    }

    /* Returns true iff the current waveform is a complete capture. */
    bool CheckComplete()
    {
        bool CaptureComplete;
        THREAD_LOCK(this);
        CaptureComplete = CapturedSamples >= (1 << AverageBits);
        THREAD_UNLOCK();
        return CaptureComplete;
    }

    /* Performs central work of accumulating a single ABCD waveform, returns
     * true iff an update to EPICS should be triggered. */
    bool AccumulateWaveform()
    {
        bool PublishUpdate;
        THREAD_LOCK(this);

        if (AverageBits > 0)
        {
            if (CapturedSamples >= (1 << AverageBits))
                ResetAccumulator();

            /* Convert IQ to ABCD and accumulate. */
            InputAbcd.CaptureCordic(WaveformIq);
            AccumulateAbcd();
            PublishUpdate = UpdateAll  ||
                CapturedSamples >= (1 << AverageBits);
        }
        else
        {
            /* Optimise special case when capturing exactly one sample:
             * bypass accumulator and capture ABCD directly. */
            WaveformAbcd.CaptureCordic(WaveformIq);
            CapturedSamples = 1;
            PublishUpdate = true;
        }
        
        THREAD_UNLOCK();
        return PublishUpdate;
    }

    
    const int WaveformLength;
    
    /* Captured and processed waveforms: these three blocks of waveforms are
     * all published to EPICS. */
    IQ_WAVEFORMS WaveformIq;
    ABCD_WAVEFORMS InputAbcd, WaveformAbcd;
    XYQS_WAVEFORMS WaveformXyqs;

    /* Offset from trigger of capture. */
    int CaptureOffset;
    
    /* Averaging control. */
    int AverageBits;            // Log2 number of samples to average
    int CapturedSamples;        // Number of samples captured so far
    bool StopWhenDone;          // Whether to restart averaging
    bool UpdateAll;             // Whether to update while averaging
    bool GotEpicsLock;          // Tracks whether we're waiting for EPICS
    UPDATER_int PublishCapturedSamples;

    /* Statistics for the captured waveforms. */
    WAVEFORM_STATS StatsX;
    WAVEFORM_STATS StatsY;

    /* Tune response measurement. */
    WAVEFORM_TUNE TuneX;
    WAVEFORM_TUNE TuneY;

    /* EPICS interlock. */
    INTERLOCK Interlock;
    ENABLE Enable;
};



static FREE_RUN * FreeRun = NULL;

bool InitialiseFreeRun(int WaveformLength)
{
    FreeRun = new FREE_RUN(WaveformLength);
    return true;
}
