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


static size_t clz_64(uint64_t x)
{
    if (x >> 32)
        return CLZ(x >> 32);
    else
        return 32 + CLZ((uint32_t) (x & 0xFFFFFFFF));
}



/* This routine computes 2^-shift * a * b with as much precision as possible.
 * This is a bit tricky, as we don't know in advance which of a and b has
 * bits to spare, so we have to normalise first. */

static int64_t LongMultiply(int64_t a, int64_t b, size_t shift)
{
    bool negative = false;
    if (a < 0) { a = - a; negative = true; }
    if (b < 0) { b = - b; negative = !negative; }

    int n = clz_64(a);
    int m = clz_64(b);
    int nm = (n + m) / 2;
    int sa = shift - n + nm;
    if (sa < 0)
        sa = 0;
    else if (sa > (int) shift)
        sa = shift;
    a >>= sa;
    b >>= shift - sa;

    int64_t ab = a * b;
    return negative ? - ab : ab;
}


WAVEFORM_TUNE::WAVEFORM_TUNE(
    XYQS_WAVEFORMS &Waveform, int Field, const char *Axis) :

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

void WAVEFORM_TUNE::Update()
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
        TotalI -= LongMultiply(cos_sum, data_sum, 16) / length;
        TotalQ -= LongMultiply(sin_sum, data_sum, 16) / length;

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


const char * WAVEFORM_TUNE::PvName(const char *Pv)
{
    return Concat("FR:TUNE", Axis, Pv);
}



WAVEFORM_STATS::WAVEFORM_STATS(
    XYQS_WAVEFORMS &Waveform, int Field, const char *Axis) :
    
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


void WAVEFORM_STATS::Update()
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
