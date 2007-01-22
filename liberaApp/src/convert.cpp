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

#include <dbFldTypes.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "cordic.h"
#include "numeric.h"
#include "interlock.h"

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


/* Control configuration. */

/* Controls the state of the rotating switches and the associated DSC daemon.
 * These are combined into a single setting here, but separated out in the
 * CSPI interface.
 *    The AutoSwitchState is set to one of the following enumeration values,
 * but has to be declared here as an integer so that it can be passed by
 * reference to the EPICS interface layer. */
enum AUTO_SWITCH_STATE
{
    /* Switches fixed to manual setting, gains set to unity. */
    AUTO_SWITCH_FIXED_UNITY = 0,
    /* Switches fixed to manual setting, gains fixed to last good setting. */
    AUTO_SWITCH_FIXED = 1,
    /* Switches automatically rotating, gains fixed to last good setting. */
    AUTO_SWITCH_ENABLED_NO_DSC = 2,
    /* Switches automatically rotating, DSC running normally. */
    AUTO_SWITCH_ENABLED_DSC = 3
};
static int AutoSwitchState = 0;
/* Selects which switch setting to use in manual mode. */
static int ManualSwitch = 3;
/* Selected attenuation.  The default is quite high for safety. */
static int CurrentAttenuation = 60;


/* Current scaling factor.  This is used to program the nominal beam current
 * for an input power of 0dBm, or equivalently, the beam current
 * corresponding to a button current of 4.5mA.
 *    This is recorded in units of 10nA, giving a maximum 0dBm current of
 * 20A. */
static int CurrentScale = 100000000;


/* Rescales value by gain factor making use of the GAIN_OFFSET defined above.
 * The gain factors are scaled by a factor of 2^31, and are intended to
 * always be <= 1.  The value to be scaled derives from a CORDIC computation,
 * and will be comfortably less that 2^31. */
#define SCALE_GAIN(gain, value) \
    ((int) (((long long int) (gain) * (value)) >> 30))


/* This flag determines the beam orientation: either diagonal or vertical.
 * Note that the Z axis (or S, as accelerator physicists call it) is *into*
 * the page, and X points out of the ring.
 * 
 *       ^ Y        A                   A       B
 *       |      D   *   B                   *    
 * X <---+-         C                   D       C
 *       |
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
 * *unsigned* multiply: an upper bound on K of over 250mm seems ample! */

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
 *              B = I * (1 - X/K)               B = I * (1 - X/K + Y/K)
 *              C = I * (1 - Y/K)               C = I * (1 - X/K - Y/K)
 *              D = I * (1 + X/K)               D = I * (1 + X/K - Y/K)
 *
 * where I is proportional to beam intensity and we are neglecting terms of
 * order X^2, Y^2 and XY.  Given this model we can calculate
 * 
 *      S = A + B + C + D = 4 * I
 *      Q = A - B + C - D = 0
 *              D_X = D - B = 2*I*X/K           D_X = A - B - C + D = 4*I*X/K
 *              D_Y = A - C = 2*I*Y/K           D_Y = A + B - C - D = 4*I*Y/K
 *
 * and thus
 *              X = 2*K * (D - B) / S           X = K * (A - B - C + D) / S
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
         *    The button values A,B,C,D are known to lie in the range 0 to
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
         * of 1/S.  (InvS,shift) = Reciprocal(S) returns InvS=2^shift/S,
         * where shift derives from a bit normalisation count on S.  Indeed,
         * we know that shift = 61-n where n is the normalisation count, so
         *      2^31 <= 2^n * S < 2^32.
         * From the observation above we know that n >= 1, and it is
         * convenient for the subsequent call to DeltaToPosition to adjust
         * things so that
         *
         *      InvS = 2^(60-shift) / S
         *      2^(30-shift) <= S < 2^(31-shift)
         */
        int shift = 0;
        int InvS = Reciprocal(S, shift);
        shift = 60 - shift;
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
            xyqs.X = DeltaToPosition(K_X, D - B, InvS, shift) << 1 - X_0;
            xyqs.Y = DeltaToPosition(K_Y, A - C, InvS, shift) << 1 - Y_0;
        }
        xyqs.Q = DeltaToPosition(K_Q, A - B + C - D, InvS, shift);
        xyqs.S = S;
    }
}



void GainCorrect(int Channel, int *Column, int Count)
{
    int Gain = ChannelGain[Channel];
    for (int i = 0; i < Count; i ++)
        Column[i] = SCALE_GAIN(Gain, Column[i]);
}



/****************************************************************************/
/*                                                                          */
/*                         Switches and Calibration                         */
/*                                                                          */
/****************************************************************************/


/* Called whenever any of the scaling calibration settings has changed.
 * These are then written to CSPI to ensure that the FPGA calculations remain
 * in step with ours. */

static void UpdateCalibration()
{
    WriteCalibrationSettings(K_X, K_Y, K_Q, X_0, Y_0);
}


/* Called whenever the autoswitch mode has changed. */

static void UpdateAutoSwitch()
{
    switch ((AUTO_SWITCH_STATE)AutoSwitchState)
    {
        case AUTO_SWITCH_FIXED_UNITY:
            /* Set the channel gains to unity, this completely disabling any
             * DSC effect, and set the selected switch position. */
            WriteDscMode(CSPI_DSC_UNITY);
            WriteSwitchState((CSPI_SWITCHMODE) ManualSwitch);
            break;
        case AUTO_SWITCH_FIXED:
            /* Turn the DSC off and set the currently selected fixed switch
             * position. */
            WriteDscMode(CSPI_DSC_OFF);
            WriteSwitchState((CSPI_SWITCHMODE) ManualSwitch);
            break;
        case AUTO_SWITCH_ENABLED_NO_DSC:
            /* Enable automatic switching with the DSC disabled, but holding
             * the last good position. */
            WriteDscMode(CSPI_DSC_OFF);
            WriteSwitchState(CSPI_SWITCH_AUTO);
            break;
        case AUTO_SWITCH_ENABLED_DSC:
            /* Enable automatic switching and turn the DSC on.  We try to
             * avoid having DSC running with switching disabled, hence the
             * change of order. */
            WriteSwitchState(CSPI_SWITCH_AUTO);
            WriteDscMode(CSPI_DSC_AUTO);
            break;
    }
}


static void UpdateManualSwitch()
{
    switch ((AUTO_SWITCH_STATE)AutoSwitchState)
    {
        case AUTO_SWITCH_FIXED_UNITY:
        case AUTO_SWITCH_FIXED:
            /* If the switches are in a fixed mode then write the new manual
             * switch setting directly to the switches. */
            WriteSwitchState((CSPI_SWITCHMODE) ManualSwitch);
            break;
        case AUTO_SWITCH_ENABLED_NO_DSC:
        case AUTO_SWITCH_ENABLED_DSC:
            /* If the switches are in automatic mode leave them alone. */
            break;
    }
}



/****************************************************************************/
/*                                                                          */
/*                          Attenuation Management                          */
/*                                                                          */
/****************************************************************************/



/* Attenuator configuration management. */


#define MAX_ATTENUATION  62
#define OFFSET_CONF_FILE "/opt/dsc/offsets.conf"


/* The attenuator value reported by ReadCachedAttenuation() is not strictly
 * accurate, due to minor offsets on attenuator values.  Here we attempt to
 * compensate for these offsets by reading an offset configuration file. */
static int AttenuatorOffset[MAX_ATTENUATION + 1];


/* This contains a precalculation of K_S * 10^((A-A_0)/20) to ensure that the
 * calculation of ComputeScaledCurrent is efficient. */
static PMFP ScaledCurrentFactor;
/* This contains a precalculation of 10^((A-A_0)/20): this only needs to
 * change when the attenuator settings are changed. */
static PMFP AttenuatorScalingFactor;


static bool ReadAttenuatorOffsets()
{
    FILE * OffsetFile = fopen(OFFSET_CONF_FILE, "r");
    if (OffsetFile == NULL)
    {
        printf("Unable to open file " OFFSET_CONF_FILE "\n");
        return false;
    }
    else
    {
        bool Ok = true;
        for (int i = 0; Ok  &&  i <= MAX_ATTENUATION; i ++)
        {
            double Offset;
            Ok = fscanf(OffsetFile, "%lf", &Offset) == 1;
            if (Ok)
                AttenuatorOffset[i] = (int) (DB_SCALE * Offset);
            else
                printf("Error reading file " OFFSET_CONF_FILE "\n");
        }
        fclose(OffsetFile);
        return Ok;
    }
}


/* Returns the current cached attenuator setting, after correcting for
 * attenuator offset. */

int ReadCorrectedAttenuation()
{
    return CurrentAttenuation * DB_SCALE +
        AttenuatorOffset[CurrentAttenuation];
}


static void UpdateCurrentScale()
{
    ScaledCurrentFactor = AttenuatorScalingFactor * CurrentScale;
}


/* Updates the attenuators and the associated current scaling factors. */

void UpdateAttenuation(int NewAttenuation)
{
    CurrentAttenuation = NewAttenuation;
    WriteAttenuation(CurrentAttenuation);
    /* Update the scaling factors. */
    AttenuatorScalingFactor = PMFP(from_dB, ReadCorrectedAttenuation() - A_0);
    UpdateCurrentScale();
}



/* Converts a raw current (or charge) intensity value into a scaled current
 * value. */

int ComputeScaledCurrent(const PMFP & IntensityScale, int Intensity)
{
    return Denormalise(IntensityScale * ScaledCurrentFactor * Intensity);
}



/****************************************************************************/
/*                                                                          */

#define PUBLISH_CALIBRATION(Name, Value) \
    PUBLISH_CONFIGURATION(Name, ao, Value, UpdateCalibration)

#define PUBLISH_GAIN(Name, Value) \
    PUBLISH_CONFIGURATION(Name, ao, Value, NULL_ACTION)


bool InitialiseConvert()
{
    if (!ReadAttenuatorOffsets())
        return false;
    
    PUBLISH_CONFIGURATION("CF:DIAG", bo, Diagonal, NULL_ACTION);

    PUBLISH_CALIBRATION("CF:KX", K_X);
    PUBLISH_CALIBRATION("CF:KY", K_Y);
    PUBLISH_CALIBRATION("CF:KQ", K_Q);
    PUBLISH_CALIBRATION("CF:X0", X_0);
    PUBLISH_CALIBRATION("CF:Y0", Y_0);

    PUBLISH_GAIN("CF:G0", ChannelGain[0]);
    PUBLISH_GAIN("CF:G1", ChannelGain[1]);
    PUBLISH_GAIN("CF:G2", ChannelGain[2]);
    PUBLISH_GAIN("CF:G3", ChannelGain[3]);

    PUBLISH_CONFIGURATION("CF:AUTOSW", mbbo,
        AutoSwitchState, UpdateAutoSwitch);
    PUBLISH_CONFIGURATION("CF:SETSW", longout,
        ManualSwitch, UpdateManualSwitch);
    PUBLISH_CONFIGURATION("CF:ISCALE", ao, CurrentScale, UpdateCurrentScale);

    /* Note that updating the attenuators is done via the
     * InterlockedUpdateAttenuation() routine: this will not actually update
     * the attenuators (by calling UpdateAttenuation() above) until the
     * interlock mechanism is ready. */
    PUBLISH_CONFIGURATION("CF:ATTEN", longout,
        CurrentAttenuation, InterlockedUpdateAttenuation);
    
    /* Write the initial state to the hardware and initialise everything that
     * needs initialising. */
    UpdateCalibration();
    WriteAgcMode(CSPI_AGC_MANUAL);
    InterlockedUpdateAttenuation(CurrentAttenuation);
    UpdateAutoSwitch();

    return true;
}
