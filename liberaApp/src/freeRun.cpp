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
    FREE_RUN_TUNE(XYQS_WAVEFORMS &Waveform, int Field, const char *Axis) :
        Waveform(Waveform),
        Field(Field),
        Axis(Axis)
    {
        Frequency = 0;
        Update();
        
        Publish_ai(PvName("I"),   I);
        Publish_ai(PvName("Q"),   Q);
        Publish_ai(PvName("MAG"), Mag);
        Publish_ai(PvName("PH"),  Phase);
        PUBLISH_METHOD_OUT(ao, PvName(""), Update, Frequency);
    }

    void Update()
    {
        if (Frequency == 0)
            /* Effectively turn processing off in this case. */
            I = Q = Mag = Phase = 0;
        else
        {
            int64_t TotalI = 0, TotalQ = 0;
            int64_t cos_sum = 0, sin_sum = 0, data_sum = 0;
            int angle = 0;
            size_t length = Waveform.GetLength();
            for (size_t i = 0; i < length; i ++)
            {
                int cos, sin;
                cos_sin(angle, cos, sin);
                cos_sum += cos;
                sin_sum += sin;
                
                int data = GET_FIELD(Waveform, i, Field, int);
                data_sum += data;
                
                /* To avoid too much loss of precision during accumulation we
                 * use a comfortable number of bits. */
                TotalI += ((int64_t) data * cos) >> 16;
                TotalQ += ((int64_t) data * sin) >> 16;
                angle += Frequency;
            }

            /* Correct for DC offset in the original cos/sin waveform: this
             * arises from the fact that there isn't (necessarily) a complete
             * cycle of the selected frequency.  So we compute
             *  I = SUM(x_i*(c_i - mean(c))) = SUM(x_i*c_i) - SUM(x)*SUM(c)/N
             * This is made a bit complicated by the fact that this is all
             * fixed point arithmetic. */
            data_sum >>= 16;
            TotalI -= (cos_sum / length) * data_sum;
            TotalQ -= (sin_sum / length) * data_sum;
            
            /* The shifts above and below add up to 28: this is 2 less than
             * the excess scaling factor 2^30 in the IQ waveform, leaving a
             * factor of 2 for CORDIC_SCALE, and a further factor of 2 to
             * convert a single frequency measurement into a properly scaled
             * magnitude. */
            I = clip(TotalI >> 12);
            Q = clip(TotalQ >> 12);

            Mag = MulUU(CordicMagnitude(I, Q), CORDIC_SCALE);
            Phase = lround(atan2(Q, I) * M_2_32 / M_PI / 2.);
            /* Finally we publish the underlying (scaled) I and Q. */
            I /= 2;
            Q /= 2;
        }
    }
    

private:
    FREE_RUN_TUNE();
    
    const char * PvName(const char *Pv)
    {
        return Concat("FR:TUNE", Axis, Pv);
    }
    

    XYQS_WAVEFORMS &Waveform;
    const int Field;
    const char *const Axis;

    int Frequency;
    int I, Q, Mag, Phase;
};



class FREE_RUN_STATS
{
public:
    FREE_RUN_STATS(XYQS_WAVEFORMS &Waveform, int Field, const char *Axis) :
        Waveform(Waveform),
        Field(Field),
        WaveformLength(Waveform.MaxLength())
    {
        Publish_ai(Concat("FR:MEAN", Axis), Mean);
        Publish_ai(Concat("FR:STD", Axis),  Std);
        Publish_ai(Concat("FR:MIN", Axis),  Min);
        Publish_ai(Concat("FR:MAX", Axis),  Max);
        Publish_ai(Concat("FR:PP", Axis),   Pp);
    }

    void Update()
    {
        long long int Total = 0;
        Min = INT_MAX;
        Max = INT_MIN;
        for (size_t i = 0; i < Waveform.GetLength(); i ++)
        {
            int Value = GET_FIELD(Waveform, i, Field, int);
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
        for (size_t i = 0; i < Waveform.GetLength(); i ++)
        {
            int64_t Value = GET_FIELD(Waveform, i, Field, int);
            Variance += (Value - Mean) * (Value - Mean);
        }
        Variance /= WaveformLength;
        /* At this point I'm lazy. */
        Std = (int) sqrt(Variance);
    }

    
private:
    FREE_RUN_STATS();
    
    XYQS_WAVEFORMS &Waveform;
    const int Field;
    const int WaveformLength;

    int Mean, Std, Min, Max, Pp;
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
    FREE_RUN_STATS StatsX;
    FREE_RUN_STATS StatsY;

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
