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


/* Implementation of Booster data support. */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
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

#include "booster.h"


#define DECIMATION      64              // Libera decimation factor


/* Fill out each axis waveform with an appropriate linear scale running
 * from 0 to Duration. */
void FillAxis(FLOAT_WAVEFORM &Axis, int Length, float Duration)
{
    float *a = Axis.Array();
    for (int i = 0; i < Length; i ++)
        a[i] = (Duration * i) / (Length - 1);
}


class BOOSTER : I_EVENT
{
public:
    BOOSTER(int ShortWaveformLength, float FRev) :
        ShortWaveformLength(ShortWaveformLength),
        LongWaveformLength(16*ShortWaveformLength),
        LongIq(LongWaveformLength),
        LongAbcd(LongWaveformLength),
        LongXyqs(LongWaveformLength),
        ShortXyqs(ShortWaveformLength),
        LongAxis(LongWaveformLength),
        ShortAxis(ShortWaveformLength)
    {
        /* Build the linear scales so that we can see booster data against a
         * sensible time scale (in milliseconds).
         *    Each point in the short waveform corresponds to 1024 points at
         * revolution frequency, hence the calculation below. */
        float RampDuration = 1024 * 1e3 * ShortWaveformLength / FRev;
        FillAxis(LongAxis,  LongWaveformLength,  RampDuration);
        FillAxis(ShortAxis, ShortWaveformLength, RampDuration);

        /* Publish the PVs associated with Booster data. */
        LongIq.Publish("BN");
        LongAbcd.Publish("BN");
        LongXyqs.Publish("BN");
        ShortXyqs.Publish("BN", "WFS");

        Publish_waveform("BN:AXIS", LongAxis);
        Publish_waveform("BN:AXISS", ShortAxis);

        /* Trigger and interlock. */
        Interlock.Publish("BN", true);
        Enable.Publish("BN");

        /* Announce our interest in the trigger. */
        RegisterTriggerEvent(*this, PRIORITY_BN);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent(int)
    {
        /* Ignore events if not enabled. */
        if (!Enable.Enabled())
            return;

        Interlock.Wait();

        LongIq.Capture(DECIMATION);
        LongAbcd.CaptureCordic(LongIq);
        LongXyqs.CaptureConvert(LongAbcd);
        ProcessShortWaveforms();

        Interlock.Ready(LongIq.GetTimestamp());
    }


private:
    void ProcessShortWaveforms()
    {
        /* Work through all of the fields in the ABCD waveform block and
         * compute a block average over each 16 points in the long waveform.
         * We write this as the fully decimated short waveform, producing a
         * total decimation of 1:1024. */
        const int Fields[] = { FIELD_A, FIELD_B, FIELD_C, FIELD_D };
        for (size_t i = 0; i < ARRAY_SIZE(Fields); i ++)
        {
            const int Field = Fields[i];
            int Long[LongWaveformLength];
            int Short[ShortWaveformLength];
            LongXyqs.Read(Field, Long, LongWaveformLength);
            /* Work through each long waveform averaging together 16
             * successive points to form one short waveform point. */
            for (int j = 0; j < ShortWaveformLength; j ++)
            {
                int Total = 0;
                for (int k = 0; k < 16; k ++)
                    Total += Long[16*j + k] >> 4;
                Short[j] = Total;
            }
            ShortXyqs.Write(Field, Short, ShortWaveformLength);
        }
    }

    /* Startup configurable dimensions. */
    const int ShortWaveformLength;
    const int LongWaveformLength;

    /* A full length waveform is captured in IQ form and convert to button
     * values and positions.  From this positions are averaged to produce a
     * short waveform of positions, with effectively one position per 1024
     * turns. */
    IQ_WAVEFORMS LongIq;
    ABCD_WAVEFORMS LongAbcd;
    XYQS_WAVEFORMS LongXyqs;
    XYQS_WAVEFORMS ShortXyqs;
    /* The two axis waveforms are used to help the display of long and short
     * waveforms in EDM by providing a time axis graduated in milliseconds. */
    FLOAT_WAVEFORM LongAxis;
    FLOAT_WAVEFORM ShortAxis;
    /* Interlock for communication with EPICS. */
    INTERLOCK Interlock;
    ENABLE Enable;
};



static BOOSTER * Booster = NULL;

bool InitialiseBooster(int ShortWaveformLength, float FRev)
{
    Booster = new BOOSTER(ShortWaveformLength, FRev);
    return true;
}
