/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2010  Michael Abbott, Diamond Light Source Ltd.
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

/* Shared support for X/Y statistics. */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "device.h"
#include "publish.h"
#include "hardware.h"
#include "waveform.h"
#include "convert.h"
#include "numeric.h"
#include "cordic.h"

#include "statistics.h"



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



static const char * PvName(const char *Group, const char *Pv, const char *Axis)
{
    char *name = (char *) malloc(
        strlen(Group) + 1 + strlen(Pv) + strlen(Axis) + 1);
    sprintf(name, "%s:%s%s", Group, Pv, Axis);
    return name;
}


STATISTICS::STATISTICS(
    const char *Group, const char *Axis,
    XYQS_WAVEFORMS &Waveform, int Field) :

    Waveform(Waveform),
    Field(Field)
{
#define PV_NAME(pv) PvName(Group, pv, Axis)
    /* Waveform statistics. */
    Mean = Std = Min = Max = Pp = 0;
    Publish_ai(PV_NAME("MEAN"), Mean);
    Publish_ai(PV_NAME("STD"),  Std);
    Publish_ai(PV_NAME("MIN"),  Min);
    Publish_ai(PV_NAME("MAX"),  Max);
    Publish_ai(PV_NAME("PP"),   Pp);

    /* Tune statistics. */
    Frequency = 0;
    UpdateTune();

    Publish_ai(PV_NAME("TUNEI"),   I);
    Publish_ai(PV_NAME("TUNEQ"),   Q);
    Publish_ai(PV_NAME("TUNEMAG"), Mag);
    Publish_ai(PV_NAME("TUNEPH"),  Phase);
    PUBLISH_METHOD_OUT(ao, PV_NAME("TUNE"), Update, Frequency);
#undef PV_NAME
}


void STATISTICS::UpdateStats()
{
    long long int Total = 0;
    Min = INT_MAX;
    Max = INT_MIN;
    size_t Length = GetLength();
    for (size_t i = 0; i < Length; i ++)
    {
        int Value = GetField(i);
        Total += Value;
        if (Value < Min)  Min = Value;
        if (Value > Max)  Max = Value;
    }
    Mean = (int) (Total / Length);
    Pp = Max - Min;

    /* We get away with accumulating the variance in a long long.  This
     * depends on reasonable ranges of values: at DLS the position is
     * +-10mm (24 bits) and the waveform is 2^11 bits long.  2*24+11 fits
     * into 63 bits, and seriously there is negligible prospect of this
     * failing anyway with realistic inputs... */
    long long int Variance = 0;
    for (size_t i = 0; i < Length; i ++)
    {
        int64_t Value = GetField(i);
        Variance += (Value - Mean) * (Value - Mean);
    }
    Variance /= Length;
    /* At this point I'm lazy. */
    Std = (int) sqrt(Variance);
}


void STATISTICS::UpdateTune()
{
    if (Frequency == 0)
        /* Effectively turn processing off in this case. */
        I = Q = Mag = Phase = 0;
    else
    {
        int64_t TotalI = 0, TotalQ = 0;
        int angle = 0;
        size_t length = GetLength();
        for (size_t i = 0; i < length; i ++)
        {
            int cos, sin;
            cos_sin(angle, cos, sin);

            /* To avoid too much loss of precision during accumulation we
             * use a comfortable number of bits. */
            int data = GetField(i) - Mean;
            TotalI += ((int64_t) data * cos) >> 16;
            TotalQ += ((int64_t) data * sin) >> 16;
            angle += Frequency;
        }

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


void STATISTICS::Update()
{
    UpdateStats();
    UpdateTune();
}
