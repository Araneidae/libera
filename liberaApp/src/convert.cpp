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

static int K_X = 10000000;    // 10mm: largely reasonable defaults
static int K_Y = 10000000;
static int K_Q = 10000000;

/* Electron beam zero point offsets.  These are used to adjust the nominal
 * zero point returned.  These are stored in nm. */
static int X_0 = 0;
static int Y_0 = 0;

/* Button gain adjustments.  By default we start with gain of 1.  See
 * SCALE_GAIN macro below. */
#define DEFAULT_GAIN    (1 << 30)
static int ChannelGain[4] =
    { DEFAULT_GAIN, DEFAULT_GAIN, DEFAULT_GAIN, DEFAULT_GAIN };
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
static bool Diagonal = true;




/*****************************************************************************/
/*                                                                           */
/*                           Conversion Routines                             */
/*                                                                           */
/*****************************************************************************/


/* The total intensity for each button is the magnitude of its IQ data: we
 * perform the reduction using cordic for which we have a very fast algorithm
 * available. */

void IQtoABCD(const IQ_ROW *IQ, ABCD_ROW *ABCD, int Count)
{
    for (int i = 0; i < Count; i ++)
    {
        const IQ_ROW & iq = IQ[i];
        ABCD_ROW & abcd = ABCD[i];
        abcd.A = CordicMagnitude(iq.AI, iq.AQ);
        abcd.B = CordicMagnitude(iq.BI, iq.BQ);
        abcd.C = CordicMagnitude(iq.CI, iq.CQ);
        abcd.D = CordicMagnitude(iq.DI, iq.DQ);
    }
}



/* Computes K * M / S without loss of precision.  We use
 * our knowledge of the arguments to do this work as efficiently as possible.
 *
 * Firstly we know that InvS = 2^(60-shift)/S and that S < 2^(31-shift).
 * However, we also know that |M| <= S (this is a simple consequence of S
 * being a sum of non-negative values and M being a sum of differences), so
 * in particular also |M| < 2^(31-shift) and so we can safely get rid of the
 * shift in InvS by giving it to M!
 *
 * We have now transformed K * M / S to 2^(60-shift) * K * M * InvS and then
 * to 2^60 * K * (2^-shift * M) * InvS.  Note finally that the scaling
 * constant K can be safely bounded by 2^27 ~~ 128mm and so we can
 * with excellent efficiency compute
 *
 *                 K * M    -64     4        shift
 *      Position = ----- = 2    * (2  K) * (2      M) * InvS
 *                   S
 * 
 * In fact we gain slightly more head-room on K by computing K*InvS as an
 * *unsigned* multiply: an upper bound on K of over 0.25m seems ample! */

static int DeltaToPosition(int K, int M, int InvS, int shift)
{
    return MulSS(MulUU(K << 4, InvS), M << shift);
}


/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function.  The underlying model for the
 * transfer of electron beam intensity to buttons simplifies to a model where
 * we can write
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
 */

void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, int Count)
{
    for (int i = 0; i < Count; i ++)
    {
        const ABCD_ROW & abcd = ABCD[i];
        XYQS_ROW & xyqs = XYQS[i];

        /* First compute the total intensity S.  To avoid overflow we
         * prescale by 4.  This can involve loss of bits when the intensity
         * is extremely low, but in fact the bottom bits are pretty well pure
         * noise and can be cheaply discarded.
         *    The button values A,B,C,D are known to line in the range 0 to
         * 2^31 - 1 so we similarly know that 0 <= S < 2^31. */
        int A = abcd.A >> 2;
        int B = abcd.B >> 2;
        int C = abcd.C >> 2;
        int D = abcd.D >> 2;
        int S = A + B + C + D;

        /* Now compute the positions according to the model.  As this is an
         * inner loop function we take some time to optimise its execution by
         * precomputing as much as possible.
         *    Start by precomputing 1/S, or more precisely, a scaled version
         * of 1/S.  (InvS,shift) = Reciprocal(S) returns InvS=2^(61-shift)/S,
         * where shift is a bit normalisation count on S, ie
         *      2^31 <= 2^shift * S < 2^32.
         * From the observation above we know that shift >= 1, and it is
         * convenient for the subsequent call to DeltaToPosition to adjust
         * things so that
         *
         *      InvS = 2^(60-shift) / S
         *      2^(30-shift) <= S < 2^(31-shift)
         */
        int shift;
        int InvS = Reciprocal(S, shift);
        shift -= 1;
        /* Compute X and Y according to the currently selected detector
         * orientation.  There seem to be no particularly meaningful
         * computation of Q in vertical orientation, so we use the diagonal
         * orientation computation for this factor. */
        if (Diagonal)
        {
            xyqs.X = DeltaToPosition(K_X, A - B - C + D, InvS, shift) - X_0;
            xyqs.Y = DeltaToPosition(K_Y, A + B - C - D, InvS, shift) - Y_0;
        }
        else
        {
            xyqs.X = DeltaToPosition(K_X, B - D, InvS, shift) << 1 - X_0;
            xyqs.Y = DeltaToPosition(K_Y, A - C, InvS, shift) << 1 - Y_0;
        }
        xyqs.Q = DeltaToPosition(K_Q, A - B + C - D, InvS, shift);
        xyqs.S = S;
    }
}



/* Rescales one row XYQS data from nm to mm. */
void XYQStomm(const XYQS_ROW &XYQSnm, XYQSmm_ROW &XYQSmm)
{
    XYQSmm.X = nmTOmm(XYQSnm.X);
    XYQSmm.Y = nmTOmm(XYQSnm.Y);
    XYQSmm.Q = nmTOmm(XYQSnm.Q);
    XYQSmm.S = XYQSnm.S;
}


void ABCDtoXYQSmm(const ABCD_ROW &ABCD, XYQSmm_ROW &XYQSmm)
{
    XYQS_ROW XYQSnm;
    ABCDtoXYQS(&ABCD, &XYQSnm, 1);
    XYQStomm(XYQSnm, XYQSmm);
}



void GainCorrect(int Channel, int *Column, int Count)
{
    int Gain = ChannelGain[Channel];
    for (int i = 0; i < Count; i ++)
        Column[i] = SCALE_GAIN(Gain, Column[i]);
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


static bool SetAtten(int Value, void * Context)
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


static bool GetAtten(int &Value, void * Context)
{
    const int Offset = (int) Context;
    ATTENUATORS attenuators;
    if (ReadAttenuators(attenuators))
    {
        Value = attenuators[Offset];
        return true;
    }
    else
        return false;
}



static bool GetSwitches(int &Value, void*)
{
    return ReadSwitches(Value);
}



static bool SetSwitches(int Value, void*)
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

    bool init(double &Result)
    {
        return read(Result);
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
    PUBLISH_DOUBLE("CF:GA", ChannelGain[0], 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GB", ChannelGain[1], 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GC", ChannelGain[2], 0, GAIN_BOUND, GAIN_SCALE);
    PUBLISH_DOUBLE("CF:GD", ChannelGain[3], 0, GAIN_BOUND, GAIN_SCALE);

    Publish_waveform("CF:ATTWF", *new GET_ATTEN_WAVEFORM);
    PUBLISH_FUNCTION_OUT(longout, "CF:ATT1", SetAtten, GetAtten, (void*)0);
    PUBLISH_FUNCTION_OUT(longout, "CF:ATT2", SetAtten, GetAtten, (void*)1);
    PUBLISH_FUNCTION_IN_OUT(longin, longout,  "CF:SW",
        GetSwitches, SetSwitches, NULL);
    return true;
}
