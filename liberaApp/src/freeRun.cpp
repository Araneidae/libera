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

#include "freeRun.h"


#define M_2_32      (4.0 * (1 << 30))

static int clip(long long int x)
{
    if (x > INT_MAX)
        return INT_MAX;
    else if (x < INT_MIN)
        return INT_MIN;
    else
        return x;
}


class FREE_RUN_TUNE
{
public:
    FREE_RUN_TUNE(int WaveformLength, const char *Axis) :
        WaveformLength(WaveformLength),
        Axis(Axis),
        RotateI(new int[WaveformLength]),
        RotateQ(new int[WaveformLength])
    {
        Frequency = 0;
        
        Publish_ai(PvName("I"),   I);
        Publish_ai(PvName("Q"),   Q);
        Publish_ai(PvName("MAG"), Mag);
        Publish_ai(PvName("PH"),  Phase);
        PUBLISH_METHOD_OUT(ao, PvName(""), SetFrequency, Frequency);
        SetFrequency();
    }

    void Update(const int *Waveform)
    {
        long long TotalI = 0, TotalQ = 0;
        for (int i = 0; i < WaveformLength; i ++)
        {
            TotalI += MulSS(4 * Waveform[i], RotateI[i]);
            TotalQ += MulSS(4 * Waveform[i], RotateQ[i]);
        }
        I = clip(TotalI);
        Q = clip(TotalQ);

        Mag = 2 * MulUU(CordicMagnitude(I, Q), CORDIC_SCALE);
        Phase = lround(atan2(Q, I) * M_2_32 / M_PI / 2.);
    }
    

private:
    FREE_RUN_TUNE();
    
    const char * PvName(const char *Pv)
    {
        return Concat("FR:TUNE", Axis, Pv);
    }
    
    void SetFrequency()
    {
        for (int i = 0, angle = 0; i < WaveformLength;
             i ++, angle += Frequency)
            cos_sin(angle, RotateI[i], RotateQ[i]);
    }
    

    const int WaveformLength;
    const char *const Axis;
    int *const RotateI, *const RotateQ; 

    int Frequency;
    int I, Q, Mag, Phase;
};



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
        TuneX(WaveformLength, "X"),
        TuneY(WaveformLength, "Y")
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

        /* Publish statistics. */
        Publish_ai("FR:MEANX", MeanX);
        Publish_ai("FR:MEANY", MeanY);
        Publish_ai("FR:STDX",  StdX);
        Publish_ai("FR:STDY",  StdY);
        Publish_ai("FR:MINX",  MinX);
        Publish_ai("FR:MINY",  MinY);
        Publish_ai("FR:MAXX",  MaxX);
        Publish_ai("FR:MAXY",  MaxY);
        Publish_ai("FR:PPX",   PpX);
        Publish_ai("FR:PPY",   PpY);

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
             * statistics and tune response measurement.
             *    I think it may be marginally faster to grab the waveform
             * once into a properly aligned local waveform, rather than having
             * to index the required field repeatedly. */
            int Waveform[WaveformLength];
            WaveformXyqs.Read(FIELD_X, Waveform, WaveformLength);
            UpdateStatistics(Waveform, MeanX, StdX, MinX, MaxX, PpX);
            TuneX.Update(Waveform);
            WaveformXyqs.Read(FIELD_Y, Waveform, WaveformLength);
            UpdateStatistics(Waveform, MeanY, StdY, MinY, MaxY, PpY);
            TuneY.Update(Waveform);

            /* Let EPICS know there's stuff to read, releases interlock. */
            Interlock.Ready(WaveformIq.GetTimestamp());
            GotEpicsLock = false;
        }
        else
            GotEpicsLock = true;
        
        PublishCapturedSamples.Write(CapturedSamples);
    }


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /* Waveform statistics.                                                  */
    
    void UpdateStatistics(
        int *Waveform, int &Mean, int &Std, int &Min, int &Max, int &Pp)
    {
        long long int Total = 0;
        Min = INT_MAX;
        Max = INT_MIN;
        for (int i = 0; i < WaveformLength; i ++)
        {
            int Value = Waveform[i];
            Total += Value;
            if (Value < Min)  Min = Value;
            if (Value > Max)  Max = Value;
        }
        Mean = (int) (Total / WaveformLength);
        Pp = Max - Min;

        /* We get away with accumulating the variance in a long long.  This
         * depends on reasonable ranges of values: at DLS the position is
         * +-10mm (24 bits) and the waveform is 2^11 bits long.  2*24+11 fits
         * into 63 bits, and seriously there is negligible prospect of this
         * failing anyway with realistic inputs... */
        long long int Variance = 0;
        for (int i = 0; i < WaveformLength; i ++)
        {
            long long int Value = Waveform[i];
            Variance += (Value - Mean) * (Value - Mean);
        }
        Variance /= WaveformLength;
        /* At this point I'm lazy. */
        Std = (int) sqrt(Variance);
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

    /* Statistics for the captured waveforms. */
    int MeanX, StdX, MinX, MaxX, PpX;
    int MeanY, StdY, MinY, MaxY, PpY;

    /* Offset from trigger of capture. */
    int CaptureOffset;
    
    /* Averaging control. */
    int AverageBits;            // Log2 number of samples to average
    int CapturedSamples;        // Number of samples captured so far
    bool StopWhenDone;          // Whether to restart averaging
    bool UpdateAll;             // Whether to update while averaging
    bool GotEpicsLock;          // Tracks whether we're waiting for EPICS
    UPDATER_int PublishCapturedSamples;

    /* Tune response measurement. */
    FREE_RUN_TUNE TuneX;
    FREE_RUN_TUNE TuneY;

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
