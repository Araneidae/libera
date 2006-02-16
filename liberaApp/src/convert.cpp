/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2006  Michael Abbott, Diamond Light Source Ltd.
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
#include <string.h>

#include <dbFldTypes.h>

#include "drivers.h"
#include "publish.h"
#include "hardware.h"
#include "cordic.h"
#include "support.h"

#include "convert.h"



/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/

/* The following global parameters are used to control the calculation of
 * electron beam position from button signal level readout. */

/* Scaling factors.  These convert relative intensities into electron beam
 * positions and are in units of distance.  The scaling factor is determined
 * by the geometry of the button or stripline assembly.
 *     These values are in units of nm and cannot be set larger than 32mm
 * without causing numerical overflow later on in the processing chain! */

int K_X = 10000000;    // 10mm: largely reasonable defaults
int K_Y = 10000000;
int K_Q = 10000000;

/* Electron beam zero point offsets.  These are used to adjust the nominal
 * zero point returned.  These are stored in nm. */
int X_0 = 0;
int Y_0 = 0;

/* Button gain adjustments.  By default we start with gain of 1.  See
 * SCALE_GAIN macro below. */
#define DEFAULT_GAIN    (1 << 30)
int G_A = DEFAULT_GAIN;
int G_B = DEFAULT_GAIN;
int G_C = DEFAULT_GAIN;
int G_D = DEFAULT_GAIN;
#define GAIN_SCALE (1.0 / DEFAULT_GAIN)

/* Rescales value by gain factor making use of the GAIN_OFFSET defined above.
 * The gain factors are scaled by a factor of 2^31, and are intended to
 * always be <= 1.  The value to be scaled derives from a CORDIC computation,
 * and will be comfortably less that 2^31. */
#define SCALE_GAIN(gain, value) \
    ((int) (((long long int) (gain) * (value)) >> 30))


/* This flag determines the beam orientation: either diagonal or vertical.
 * 
 *                  A                   B       A
 *              D   *   B                   *    
 *                  C                   C       D
 *
 *              Vertical                Diagonal
 *
 * The default configuration is diagonal: this is the normal arrangement with
 * buttons in an oblong cross-section vacuum vessel for a synchrotron ring.
 * The vertical configuration can arise when buttons or striplines are
 * arranged around a circular vacuum vessel in a linear accelerator or
 * transfer path. */
bool Diagonal = true;




/*****************************************************************************/
/*                                                                           */
/*                           Conversion Routines                             */
/*                                                                           */
/*****************************************************************************/


/* The calculation here is a rather delicate matter.  We have already
 * computed the total button intensities 
 *      Intensity = S = A + B + C + D
 * and the appropriate X/Y offset calculation
 *      Delta = M = A - B - C + D or A + B - C - D .
 * We are also given a scaling factor
 *      Scaling = K
 * which determines the nominal offset corresponding to M = S.  Naively we
 * simply want to return
 *
 *      Position = (K * M) / S .
 *
 * The given values have already been scaled appropriately: automatic gain
 * control should normally maintain S somewhere close to 2^30, and the
 * scaling factor is in units of distance.
 *     Our geometry puts K close to 20mm, and as positions will be returned
 * as integers scaled to nm this means that K is close to 2^24.
 *
 * The obvious approach is to return
 *      (int) (((long long) a * b) / c) ,
 * but there are two reasons why this isn't done:
 *  -  Division on the ARM is relatively costly, and in particular double
 *     length division is doubly expensive;
 *  -  The gcc 3.3.3 implementation of 64 bit division (a call to __divdi3)
 *     seems to be comprehensively broken: it causes crashes!
 *
 * We therefore rely on 32/32 bit division together with a lot of careful
 * prescaling to ensure that we get as close to 16 bits final precision in
 * the final result. */


/* Generic utility for computing X,Y,Q from a Scaling factor K, the computed
 * Delta M and a total Intensity S.  This computes
 *      (K * M) / S .
 * There is an assumption (which is *not* checked) that the Scaling factor is
 * strictly smaller than 2^25: this corresponds to an overall scaling factor
 * of 32mm.  If K is too large then the results returned will be rubbish, as
 * internal overflow will occur. */

int DeltaToPosition(int Scaling, int Delta, int Intensity)
{
    bool Negative = false;
    if (Delta < 0)
    {
        Negative = true;
        Delta = - Delta;
    }

    /* Now work with unsigned longs throughout.  We're pushing the boundaries
     * of the words throughout, so this is necessary. */
    unsigned int K = Scaling;
    unsigned int M = Delta;
    unsigned int S = Intensity;

    /* If M is larger than S then we might as well overflow right away.  On
     * overflow we might as well return 0: it's harmless, and the value isn't
     * very meaningful anyway. */
    if (M >= S)  return 0;

    /* First normalise S and M together.  This won't affect anything further,
     * and ensures we use as many bits as possible. */
    int Shift = CLZ(S);
    S <<= Shift;
    M <<= Shift;

    /* Also normalise M.  This allows us to maintain a dynamic precision of
     * up to 16 bits in the final result.  In this case we need to keep count
     * as the final result is affected.  Here we can't shift by more than 9,
     * as we need to correct this shift later. */
    Shift = CLZ(M);
    if (Shift > 9)  Shift = 9;
    M <<= Shift;
    /* Finally compute the remaining shift. */
    Shift = 9 - Shift;
    
    /* Nearly there.  We now need to scale everything to ensure that we both
     * support the required dynamic range and the required precision.  We can
     * hope for at most 16 bits of precision in the result when doing 32 bit
     * division, as the precision needs to be shared between divisor and
     * dividend.  Our constraints are:
     *
     *  X       Want at least 0.1um precision
     *          Maximum scale is around +- 4mm
     *  K       This scaling factor can be bounded by 2^24 and needs little
     *          precision
     *  M       We have scaled M to be close to S
     *
     * We need to ensure that K*M < 2^32 and that S is 16 bits long.  Our
     * limits on K and M are 2^25 and 2^32 respectively, so we need to take 25
     * bits away from KM.  We split them almost equally, as M has more to
     * spare and K doesn't need them: we still have 20 bits left for M.
     *
     * In anticipation of this we've already prescaled M by some factor k=9-s
     * where s is the shift remaining.  We now compute
     *
     *                -13     k-12
     *           s   2   K * 2    M    s-13+k-12+16 KM
     *      X = 2  * -------------- = 2             -- = KM/S
     *                    -16                       S
     *                   2   S
     */
    K >>= 13;
    M >>= 12;
    S >>= 16;

    /* If S has evaporated then we're in danger of overflow. */
    if (S < ((unsigned int) 1 << Shift))  return 0;
    
    unsigned int X = (K * M + (S >> 1)) / S;
    X <<= Shift;
    
    if (Negative)
        return - (int) X;
    else
        return (int) X;
}




/* Compute magnitude of each button and scale by gain factor. */

void SinCosToABCD(LIBERA_ROW * Rows, size_t Length)
{
    for (size_t i = 0; i < Length; i ++)
    {
        LIBERA_ROW & Row = Rows[i];
        Row[0] = SCALE_GAIN(G_A, CordicMagnitude(Row[0], Row[1]));
        Row[1] = SCALE_GAIN(G_B, CordicMagnitude(Row[2], Row[3]));
        Row[2] = SCALE_GAIN(G_C, CordicMagnitude(Row[4], Row[5]));
        Row[3] = SCALE_GAIN(G_D, CordicMagnitude(Row[6], Row[7]));
    }
}



/* The underlying model for the transfer of electron beam intensity to
 * buttons simplifies to a model where we can write
 *
 *              Vertical                        Diagonal
 *              
 *              A = I * (1 + Y/K)               A = I * (1 + X/K + Y/K)
 *              B = I * (1 + X/K)               B = I * (1 - X/K + Y/K)
 *              C = I * (1 - Y/K)               C = I * (1 - X/K - Y/K)
 *              D = I * (1 - X/K)               D = I * (1 + X/K - Y/K)
 *
 * where I is proportional to beam intensity and we are neglecting terms of
 * order X^2, Y^2 and XY.  Given this model we can calculate
 * 
 *      S = A + B + C + D = 4 * I
 *      Q = A - B + C - D = 0
 *              D_X = B - D = 2*I*X/K           D_X = A - B - C + D = 4*I*X/K
 *              D_Y = A - C = 2*I*Y/K           D_Y = A + B - C - D = 4*I*Y/K
 *
 * and thus
 *              X = 2*K * (B - D) / S           X = K * (A - B - C + D) / S
 *              Y = 2*K * (A - C) / S           X = K * (A + B - C - D) / S .
 *
 * This routine performs these calculations on an array of rows, reading the
 * button values A, B, C, D from entries 0 to 3 and writing X, Y, Q, S into
 * entries 4 to 7 respectively. */

void ABCDtoXYQS(LIBERA_ROW * Rows, size_t Length)
{
    for (size_t i = 0; i < Length; i ++)
    {
        LIBERA_ROW & Row = Rows[i];
        
        /* Extract the four button values.  Scale by 1/4 to avoid any hazard
         * of overflow in what follows. */
        int A = Row[0] >> 2;
        int B = Row[1] >> 2;
        int C = Row[2] >> 2;
        int D = Row[3] >> 2;
        int S = A + B + C + D;

        if (Diagonal)
        {
            Row[4] = DeltaToPosition(K_X, A - B - C + D, S) - X_0;
            Row[5] = DeltaToPosition(K_Y, A + B - C - D, S) - Y_0;
        }
        else
        {
            Row[4] = DeltaToPosition(K_X, B - D, S) << 1 - X_0;
            Row[5] = DeltaToPosition(K_Y, A - C, S) << 1 - Y_0;
        }

        Row[6] = DeltaToPosition(K_Q, A - B + C - D, S);
        Row[7] = S;
    }
}



/****************************************************************************/
/*                                                                          */
/*                        Attenuators and Switches                          */
/*                                                                          */
/****************************************************************************/

/* The following functions are concerned with setting and reading the
 * attenuators and switch settings on the Libera. */


/* The eight attenuator settings are read as a complete waveform but written
 * as a pair of settings across all four channels. */


class GET_ATTEN_WAVEFORM : public I_WAVEFORM
{
public:
    GET_ATTEN_WAVEFORM() : I_WAVEFORM(DBF_LONG) { }
    
    size_t read(void * array, size_t length)
    {
        ATTENUATORS attenuators;
        if (ReadAttenuators(attenuators))
        {
            const size_t LENGTH = 8;    // Number of attenuators 
            if (length > LENGTH)  length = LENGTH;
            int * target = (int *) array;
            for (size_t i = 0; i < length; i ++)
                target[i] = attenuators[i];
            return length;
        }
        else
            return 0;
    }
};


bool SetAtten(int Value, void * Context)
{
    const int Offset = (int) Context;
    if (0 <= Value  &&  Value < 32)
    {
        ATTENUATORS attenuators;
        if (ReadAttenuators(attenuators))
        {
            for (int i = 0; i < 4; i ++)
                attenuators[Offset + 2*i] = (unsigned char) Value;
            return WriteAttenuators(attenuators);
        }
        else
            return false;
    }
    else
    {
        printf("Attenuator value %d out of range\n", Value);
        return false;
    }
}



bool GetSwitches(int &Value, void*)
{
    return ReadSwitches(Value);
}



bool SetSwitches(int Value, void*)
{
    return WriteSwitches(Value);
}




/****************************************************************************/
/*                                                                          */
/*                          Publishing Parameters                           */
/*                                                                          */
/****************************************************************************/

/* Class to publish a writeable int to EPICS to appear as both an ai and ao
 * value.  When read and written the value is automatically rescaled between
 * double and integer. */

class CONFIG_DOUBLE : public I_ai, public I_ao
{
public:
    CONFIG_DOUBLE(
        const char * Name,
        int &Parameter, int LowLimit, int HighLimit, double Scale) :

        Name(Name),
        Parameter(Parameter),
        LowLimit(LowLimit * Scale),
        HighLimit(HighLimit * Scale),
        Scale(Scale)
    {
        Publish_ai(Name, *this);
        Publish_ao(Name, *this);
    }

    bool read(double &Result)
    {
        Result = Scale * Parameter;
        return true;
    }

    bool write(double Value)
    {
        if (LowLimit < Value  &&  Value < HighLimit)
        {
            Parameter = (int) (Value / Scale);
            return true;
        }
        else
        {
            printf("Value of %g for parameter %s is out of range\n",
                Value, Name);
            return false;
        }
    }

private:
    const char * Name;
    int &Parameter;
    const double LowLimit;
    const double HighLimit;
    const double Scale;
};


#define PUBLISH_DOUBLE(args...) new CONFIG_DOUBLE(args)


#define SCALE 1e-6
#define BOUND (1 << 25)         // 2^25nm or approx 32mm
#define GAIN_BOUND (DEFAULT_GAIN + DEFAULT_GAIN/2)

bool InitialiseConvert()
{
    Publish_bi("CF:DIAG", Diagonal);
    Publish_bo("CF:DIAG", Diagonal);
    PUBLISH_DOUBLE("CF:KX", K_X, 0, BOUND, SCALE);
    PUBLISH_DOUBLE("CF:KY", K_Y, 0, BOUND, SCALE);
    PUBLISH_DOUBLE("CF:KQ", K_Q, 0, BOUND, SCALE);
    PUBLISH_DOUBLE("CF:X0", X_0, -BOUND/2, BOUND/2, SCALE);
    PUBLISH_DOUBLE("CF:Y0", Y_0, -BOUND/2, BOUND/2, SCALE);
    PUBLISH_DOUBLE("CF:GA", G_A, 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GB", G_B, 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GC", G_C, 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GD", G_D, 0, GAIN_BOUND, GAIN_SCALE);

    Publish_waveform("CF:ATTWF", *new GET_ATTEN_WAVEFORM);
    PUBLISH_FUNCTION(longout, "CF:ATT1", SetAtten, (void*)0);
    PUBLISH_FUNCTION(longout, "CF:ATT2", SetAtten, (void*)1);
    PUBLISH_FUNCTION(longin,  "CF:SW", GetSwitches, NULL);
    PUBLISH_FUNCTION(longout, "CF:SW", SetSwitches, NULL);
    return true;
}
