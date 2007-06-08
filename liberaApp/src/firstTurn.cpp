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


/* Implementation of First Turn support. */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>       // Note: only used during initialisation

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "configure.h"
#include "waveform.h"
#include "numeric.h"
#include "cordic.h"

#include "firstTurn.h"


/* Recorded S level at 45dB attenuation and input power 0dBm.  The ratio
 * between this value and the corresponding value in slowAcquisition.cpp
 * determines the scaling between SA and FT current measurements.  This ratio
 * should be purely a function of the FPGA signal processing chain. */
/* Note: this figure is going to have to become configurable, as different
 * versions of Libera have different scaling factors here! */
#define S_0             321260


/* Dual of offsetof macro, used to reference a selected field. */
#define use_offset(Type, Struct, Field) \
    (*(Type *)((char *)&(Struct) + (Field)))



/* Processing the raw ADC data is a suprisingly complicated process.  The
 * following routines capture the stages. */

/* After processing we work with ADC data in a sign extended and transposed
 * form. */
typedef int EXTRACTED_ADC[4][ADC_LENGTH];

/* A permutation is a mapping from channel to button. */
typedef unsigned int PERMUTATION[4];


/* Array of offsets into ABCD_ROW structure. */
static const size_t AbcdFields[] = { FIELD_A, FIELD_B, FIELD_C, FIELD_D };

/* This array translates switch positions into button permutations.  This is
 * needed when reading raw ADC buffers to undo the permutation performed by
 * the input switch. */
static const PERMUTATION PermutationLookup[16] =
{
    { 0, 1, 2, 3 },  { 0, 2, 1, 3 },  { 3, 1, 2, 0 },  { 3, 2, 1, 0 },
    { 0, 1, 3, 2 },  { 0, 2, 3, 1 },  { 3, 1, 0, 2 },  { 3, 2, 0, 1 },
    { 1, 0, 2, 3 },  { 2, 0, 1, 3 },  { 1, 3, 2, 0 },  { 2, 3, 1, 0 },
    { 1, 0, 3, 2 },  { 2, 0, 3, 1 },  { 1, 3, 0, 2 },  { 2, 3, 0, 1 }
};



/* Returns the maximum ADC value. */

static int Maximum(EXTRACTED_ADC &Data)
{
    int Maximum = 0;
    for (int Index = 0; Index < 4; Index ++)
    {
        for (int i = 0; i < ADC_LENGTH; i ++)
        {
            int x = Data[Index][i];
            if ( x > Maximum)  Maximum = x;
            if (-x > Maximum)  Maximum = -x;
        }
    }
    return Maximum;
}
    

/* Extracts the raw data from an ADC data block and publishes it to the given
 * waveform.  The data is sign extended and transposed for convenience in
 * subsequent processing. */

static void ExtractRawData(
    ADC_DATA &RawData, EXTRACTED_ADC &Extracted, ABCD_WAVEFORMS &RawAdc)
{
    /* Extract, sign extend from 12 bits to 32 bits, and transpose. */
    for (int i = 0; i < ADC_LENGTH; i ++)
        for (int j = 0; j < 4; j ++)
            Extracted[j][i] = ((int) RawData[i][j] << 20) >> 20;
    /* Publish each column to RawAdc. */
    for (int j = 0; j < 4; j ++)
        RawAdc.Write(AbcdFields[j], Extracted[j], ADC_LENGTH);
}


/* This stage of processing the ADC data takes advantage of a couple of
 * important features of the data being sampled.  The input signal is RF (at
 * approximately 500MHz) and is undersampled (at approximately 117Mhz) so that
 * the centre frequency appears at close to 1/4 the sampling frequency.  To
 * make this possible, the signal is filtered through a narrow band
 * (approximately 10MHz bandwith) filter.
 *
 * Thus the intensity profile of the incoming train can be recovered by the
 * following steps:
 *  - mix with the centre frequency (producing a complex IQ waveform) to
 *    bring the carrier frequency close to DC
 *  - low pass filter the data
 *  - compute the absolute magnitude of the waveform.
 * This is essentially the work that is performed in the FPGA.
 *
 * Furthermore, because the carrier frequency is so close to 1/4 sampling
 * frequency, mixing can be simplified here to a matter of multiplying
 * successively by exp(pi*i*n/2), in other words, by the sequence
 *      1,  i,  -1,  -i,  1,  ...
 * and if we then low pass filter by averaging four points together before
 * computing the magnitude, we can reduce the data stream
 *      x1, x2, x3, x4, x5, ...
 * to the stream
 *      |(x1-x3,x2-x4)|, |(x5-x7,x6-x8)|, ...
 * Of course, we know how to compute |(x,y)| with great efficiency.
 *
 * At the same time we rescale the data to lie in the range 0..2^30. */
static void CondenseAdcData(
    const int Raw[ADC_LENGTH], int Condensed[ADC_LENGTH/4])
{
    const int *p = Raw;
    int *q = Condensed;
    for (int i = 0; i < ADC_LENGTH; i += 4)
    {
        int x1 = *p++, x2 = *p++, x3 = *p++, x4 = *p++;
        /* Scale the raw values so that they're compatible in magnitude with
         * turn-by-turn filtered values.  This means that we can use the same
         * scaling rules downstream.
         *    A raw ADC value is +-2^11, and by combining pairs we get +-2^12.
         * Scaling by 2^18 gives us values in the range +-2^30, which will be
         * comfortable.  After cordic this becomes 0..sqrt(2)*0.5822*2^30 or
         * 0.823*2^30. */
        *q++ = CordicMagnitude((x1-x3)<<18, (x2-x4)<<18);
    }
}



/* provides support for "first turn" data.  This uses triggered data to read
 * a short waveform which is then converted into X,Y,S,Q values locally. */

class FIRST_TURN : I_EVENT
{
public:
    FIRST_TURN(int Harmonic, int Decimation) :
        PersistentOffset(Offset),
        PersistentLength(Length),
        RawAdc(ADC_LENGTH),
        Adc(ADC_LENGTH/4),
        ChargeScale(PMFP(10) / (PMFP(S_0) * 117))
    {
        /* Sensible defaults for offset and length.  Must be bounded to lie
         * within the waveform.
         *    By default we set the window to cover approximately two bunches
         * at booster clock frequency.  This allows a sensible signal to be
         * read without tight adjustment of the timing.  At the Libera sample
         * clock rate of approximately 117MHz, and with four raw ADC points
         * per processed sample, each point corresponds to approximately
         * 34ns. */
        Offset = 5;
        Length = 31;

        InitialiseRotation(Harmonic, Decimation);
        
        /* Now initialise the persistence of these and initialise the length
         * state accordingly. */
        PersistentOffset.Initialise("FT:OFF");
        PersistentLength.Initialise("FT:LEN");
        SetLength(Length);
        
        /* Computed button totals and associated button values. */
        RawAdc.PublishRaw("FT");
        Adc.Publish("FT");
        Publish_ABCD("FT", ABCD);
        Publish_XYQS("FT", XYQS);

        /* The computed maximum ADC values (used to detect overflow). */
        Publish_longin("FT:MAXADC", MaxAdc);
        /* The integrated charge. */
        Publish_ai("FT:CHARGE", Charge);

        /* Finally the trigger used to notify events.  The database wires this
         * up so that the all the variables above are processed when a trigger
         * has occured.  This code is then responsible for ensuring that all
         * the waveforms are updated before the trigger is updated.   */
        Interlock.Publish("FT");
        Enable.Publish("FT");

        /* Also publish access to the offset and length controls for the
         * averaging window. */
        PUBLISH_METHOD_OUT(longout, "FT:OFF", SetOffset, Offset);
        PUBLISH_METHOD_OUT(longout, "FT:LEN", SetLength, Length);

        /* Announce our interest in the trigger event from libera. */
        RegisterTriggerEvent(*this, PRIORITY_FT);
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

        /* Read and process the ADC waveform into ABCD values and extract the
         * raw integrated charge. */
        int RawCharge = ProcessAdcWaveform();
        /* Convert button values to XYQS values. */
        ABCDtoXYQS(&ABCD, &XYQS, 1);
        /* Convert raw charge into displayable value in proper units. */
        Charge = ComputeScaledCurrent(ChargeScale, RawCharge);

        /* Finally tell EPICS there's stuff to read. */
        Interlock.Ready();
    }

    
private:
    FIRST_TURN();
    
    /* Performs the fairly complex processing required to convert a raw ADC
     * waveform into published waveform and button values.  We perform the
     * following stages of processing:
     *
     *  1. Read the raw waveforms directly from hardware.
     *  2. Sign extend and extract into separate arrays.  These can then be
     *     written to RawAdc to be published to EPICS as desired.
     *  3. Condense each 1024 sample raw column into a 256 sample column.
     *     This involves filtering tricks and takes advantage of the
     *     structure of the raw data.
     *  4. Gain correct each column.
     *  5. Permute the columns according to the currently selected switch and
     *     write into Adc to be published to EPICS.
     *  6. Extract the integrated ABCD values from the permuted column.
     *  7. Finally compute XYQS. */
    int ProcessAdcWaveform()
    {
        const PERMUTATION & Permutation =
            PermutationLookup[ReadSwitchSetting()];
        ADC_DATA RawData;
        ReadAdcWaveform(RawData);
        
        /* Extract into arrays, sign extend, transpose and publish. */
        EXTRACTED_ADC Extracted;
        ExtractRawData(RawData, Extracted, RawAdc);
        MaxAdc = Maximum(Extracted);
        
        /* Now work through each column and condense it, gain correct, and
         * publish it. */
        int RawCharge = 0;
        for (int i = 0; i < 4; i ++)
        {
            /* One complication here is correcting for the input multiplexor
             * switch.  The gain settings and raw ADC readings are per
             * processing channel, bug after condensing and gain correction
             * we want to undo the switch permutation so that the button
             * readings appear in the correct sequence. */
            int Channel = Permutation[i];
            size_t Field = AbcdFields[i];
            int Condensed[ADC_LENGTH/4];
            CondenseAdcData(Extracted[Channel], Condensed);
            GainCorrect(Channel, Condensed, ADC_LENGTH/4);
            Adc.Write(Field, Condensed, ADC_LENGTH/4);
            
            /* Also update the appropriate field of the ABCD structure.  Note
             * that we use different algorithms for computing button
             * intensities and estimating the charge: it turns out that
             * IntegrateIntensity() is better at position calulations, but
             * much worse at computing charge (train length and profile has
             * too much effect). */
            use_offset(int, ABCD, Field) = IntegrateIntensity(Condensed);
            /* Finally accumulate an integrated charge. */
            RawCharge += IntegrateCharge(Extracted[Channel]);
        }
        return RawCharge;
    }


    
    /* Function for computing the total charge (in arbitrary units) coming
     * into a button.
     *     The calculation here is similar in spirit to the calculation done in
     * CondenseAdcData above.  The essential point is that integrating the
     * frequency shifted waveform will give us a true estimate of the charge in
     * the bunches which generated the waveform.
     *     To be accurate we need to shift by the true frequency offset rather
     * than by 0.25: although the difference is small, it can make a large
     * difference to the calculated charge. */
    int IntegrateCharge(const int Data[])
    {
        int TotalI = 0;
        int TotalQ = 0;
        for (int i = 4*Offset; i < 4*(Offset + Length); i++)
        {
            /* Let's do some bit arithmetic.  Each point is sign + 11 bits, we
             * are accmulating 1024 samples on each of four buttons (that's 12
             * bits) and the rotations are 30 bits plus sign (well, there's a
             * boundary condition where the 31st bit gets used, but we don't
             * need to worry too much about that): that's 53 bits plus sign.
             * MulSS will discard 32 bits, and as we want the result to fit
             * into 31 bits plus sign we want an extra 10 bits. */
            int point = Data[i] << 10;
            TotalI += MulSS(point, RotateI[i]);
            TotalQ += MulSS(point, RotateQ[i]);
        }
        return CordicMagnitude(TotalI, TotalQ);
    }


    /* The total charge in a train of bunches is directly proportional to the
     * intensity of the RF line of the sampled waveform.  We compute this as
     * the sum
     *              | /                           |
     *          Q = | | w(t) exp(2 pi i f_0 t) dt |
     *              | /                           |
     *
     * where w_0 is the frequency offset as a fraction of the sample
     * frequency.  In practice f_0 is very close to 1/4, but the difference
     * makes a significant difference.  Thus this routine precomputes the
     * expression exp(2 pi i f_0 t) = cos(w_0 t) + i sin(w_0 t), w_0=2 pi f_0.
     *
     * The two arrays (real and imaginary, or I and Q) are scaled by 2^30.
     *
     * The parameter Harmonic is the number of bunches in a machine
     * revolution, while Decimation is the number of samples in a revolution
     * (936/220 for the Diamond storage ring, 264/62 for the Diamond
     * booster).  These directly determine f_0. */
    void InitialiseRotation(int Harmonic, int Decimation)
    {
        const double Offset = (double) (Harmonic % Decimation);
        const double w_0 = 2.0 * M_PI * Offset / Decimation;
        const double Scaling = (double) (1 << 30);
        for (int i = 0; i < ADC_LENGTH; i ++)
        {
            RotateI[i] = lround(Scaling * cos(i * w_0));
            RotateQ[i] = lround(Scaling * sin(i * w_0));
        }
    }



    /* The button intensity is estimated simply by integrating the processed
     * ADC waveform.  We could use IntegrateCharge() to compute this value,
     * but that generally results in signifcantly more noise.  On the other
     * hand, we could integrate power as a proxy for intensity, taking square
     * roots at the end, but that doesn't gain a that much and is a good deal
     * more work. */
    int IntegrateIntensity(const int Data[])
    {
        int Total = 0;
        for (int i = 0; i < Length; i ++)
            /* Prescale data as we accumulate to ensure we don't overflow.  We
             * know that each data point is <2^30 (see the Capture() method of
             * ADC_WAVEFORM), and there can be at most 2^8 points, so scaling
             * by 2^-7 is safe.
             *    Fortunately there are plenty of spare bits at the bottom of
             * each sample, so we can afford to spend seven of them here! */
            Total += Data[Offset + i] >> 7;
        return Total;
    }



    /* Access methods for offset and length. */
    bool SetOffset(int newOffset)
    {
        if (0 <= newOffset  &&  newOffset + Length <= ADC_LENGTH/4)
        {
            Offset = newOffset;
            return true;
        }
        else
        {
            printf("Offset setting %d too large (length = %d)\n",
                newOffset, Length);
            return false;
        }
    }

    bool SetLength(int newLength)
    {
        if (0 < newLength  &&  Offset + newLength <= ADC_LENGTH/4)
        {
            Length = newLength;
            return true;
        }
        else
        {
            printf("Length setting %d too large (offset = %d)\n",
                newLength, Offset);
            return false;
        }
    }


    /* Control variables for averaging defining the offset into processed ADC
     * buffer and the length of the averaging window. */
    int Offset, Length;
    PERSISTENT_INT PersistentOffset, PersistentLength;
    
    /* Computed state.  The button values are integrated from the selection of
     * points, and the appropriate elements are published to epics. */
    ABCD_WAVEFORMS RawAdc;
    ABCD_WAVEFORMS Adc;
    ABCD_ROW ABCD;
    XYQS_ROW XYQS;

    /* Maximum raw ADC across all four buttons. */
    int MaxAdc;
    /* Integrated charge corresponding to measured S. */
    int Charge;

    /* Epics trigger and interlock. */
    INTERLOCK Interlock;
    ENABLE Enable;


    /* Scaling constant for charge.
     *
     * If we write the charge as
     * 
     *          /
     *      Q = | I dt = SUM I(S/S_0) Dt
     *          /
     *
     * where I(S/S_0) = ComputeScaledCurrent(1/S_0, S) and Dt = 1 / 117MHz
     * (sample frequency) then by taking unit scaling into account we can
     * determine K = ChargeScale.  The operation I(K,S) is bilinear, so we can
     * also write
     *
     *      Q = I(Dt/S_0, SUM S)
     *  
     * The units of I are 10nA, ie 10^-8 A and we'll display Q in units of
     * 10^-15 Coulombs (so giving a full scale range of 2 microculombs).
     * Thus we have:
     *
     *            15    -8   (    1       1        )
     *      Q = 10  * 10  * I(--------- * --, SUM S)
     *                       (        6   S        )
     *                       (117 * 10     0       )
     * and thus
     *
     *      K = 10 / (117 * S_0)
     *      
     * and we can compute
     *
     *      Q = I(K, SUM S)
     */
    const PMFP ChargeScale;

    /* Precomputed rotation vector (I & Q components) for frequency shifting
     * sampled waveform for charge computation. */
    int RotateI[ADC_LENGTH];
    int RotateQ[ADC_LENGTH];
};



static FIRST_TURN * FirstTurn = NULL;

bool InitialiseFirstTurn(int Harmonic, int Decimation)
{
    FirstTurn = new FIRST_TURN(Harmonic, Decimation);
    return true;
}
