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


/* Provide support for free running "turn by turn" data.  This data is
 * acquired continously. */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"

#include "freeRun.h"



class FREE_RUN : I_EVENT
{
public:
    FREE_RUN(int WaveformLength) :
        WaveformLength(WaveformLength),
        WaveformIq(WaveformLength),
        WaveformAbcd(WaveformLength),
        WaveformXyqs(WaveformLength)
    {
        CaptureOffset = 0;
        
        /* Publish all the waveforms and the interlock. */
        WaveformIq.Publish("FR");
        WaveformAbcd.Publish("FR");
        WaveformXyqs.Publish("FR");
        Publish_longout("FR:DELAY", CaptureOffset);

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
    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed.
     *    We only process if armed. */
    void OnEvent(int)
    {
        /* Ignore events if not enabled. */
        if (!Enable.Enabled())
            return;
        
        /* Wait for EPICS to be ready. */
        Interlock.Wait();

        /* Capture and convert everything. */
        WaveformIq.Capture(1, CaptureOffset);
        WaveformAbcd.CaptureCordic(WaveformIq);
        WaveformXyqs.CaptureConvert(WaveformAbcd);

        UpdateStatistics(FIELD_X, MeanX, StdX, MinX, MaxX, PpX);
        UpdateStatistics(FIELD_Y, MeanY, StdY, MinY, MaxY, PpY);

        /* Let EPICS know there's stuff to read. */
        Interlock.Ready(WaveformIq.GetTimestamp());
    }


    void UpdateStatistics(
        int Field, int &Mean, int &Std, int &Min, int &Max, int &Pp)
    {
        /* I think it may be marginally faster to grab the waveform once into
         * a properly aligned local waveform, rather than having to index the
         * required field repeatedly. */
        int Waveform[WaveformLength];
        WaveformXyqs.Read(Field, Waveform, WaveformLength);
        
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
            long long int Value = use_offset(int, Waveform[i], Field);
            Variance += (Value - Mean) * (Value - Mean);
        }
        Variance /= WaveformLength;
        /* At this point I'm lazy. */
        Std = (int) sqrt(Variance);
    }


    const int WaveformLength;
    
    /* Captured and processed waveforms: these three blocks of waveforms are
     * all published to EPICS. */
    IQ_WAVEFORMS WaveformIq;
    ABCD_WAVEFORMS WaveformAbcd;
    XYQS_WAVEFORMS WaveformXyqs;

    /* Statistics for the captured waveforms. */
    int MeanX, StdX, MinX, MaxX, PpX;
    int MeanY, StdY, MinY, MaxY, PpY;

    /* Offset from trigger of capture. */
    int CaptureOffset;

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
