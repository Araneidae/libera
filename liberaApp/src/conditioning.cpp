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

/* Signal conditioning interface. */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>

#include <dbFldTypes.h>         // DBF_UCHAR
#include <iocsh.h>
#include <epicsExport.h>

#include "complex.h"
#include "device.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "persistent.h"
#include "trigger.h"
#include "waveform.h"

#include "conditioning.h"


/* There are two standard switch sequences that we use: an 8 round sequence
 * for Libera Electron, and a 4 round sequence for Libera Brilliance. */
static const char ElectronSwitchSequence[8]   = { 3, 7, 15, 11, 0, 4, 12, 8 };
static const char BrillianceSwitchSequence[4] = { 15, 0, 9, 6 };

/* This array translates switch positions into button permutations.  This is
 * needed when reading raw ADC buffers to undo the permutation performed by
 * the input switch.
 *    For each permutuation row the entry p[b] determines which ADC channel
 * is processing the signal for button b. */
static const PERMUTATION PermutationLookup[] =
{
    { 3, 2, 1, 0 },  { 3, 1, 2, 0 },  { 0, 2, 1, 3 },  { 0, 1, 2, 3 },
    { 3, 2, 0, 1 },  { 3, 1, 0, 2 },  { 0, 2, 3, 1 },  { 0, 1, 3, 2 },
    { 2, 3, 1, 0 },  { 1, 3, 2, 0 },  { 2, 0, 1, 3 },  { 1, 0, 2, 3 },
    { 2, 3, 0, 1 },  { 1, 3, 0, 2 },  { 2, 0, 3, 1 },  { 1, 0, 3, 2 }
};


/* Some magic numbers to be configurable real soon now. */
#define SWITCH_PERIOD   40
#define SWITCH_HOLDOFF  10
#define SAMPLE_SIZE     2048
#define PRESCALE        8

#define AI_SCALE        1e6

/* We'll try to distinguish iterations over channels from iterations over
 * buttons.  Button indexes will be named b or i, channel indexes will be c
 * or j. */
#define CHANNEL_COUNT   4


/* Control configuration. */

/* Controls the rotating switches: manual or automatic mode. */
static bool AutoSwitchState = false;
// /* Selects which switch setting to use in manual mode. */
static int ManualSwitch = 3;
/* This is the currently programmed sequence of switches. */
static const char * SwitchSequence;
static int SwitchSequenceLength;



/*****************************************************************************/
/*                                                                           */
/*                Miscellaneous Helper Routines and Definitions              */
/*                                                                           */
/*****************************************************************************/


// /* Selects internal or external triggering for the rotating switches. */
// static bool ExternalSwitchTrigger = false;
// /* Selects the delay from external trigger for the switches. */
// static int SwitchTriggerDelay = 0;
// 
// /* Controls the Digital Signal Conditioning daemon state. */
// static int DscState = 0;
// static READBACK<int> * DscReadback = NULL;


typedef complex COMPENSATION_MATRIX[CHANNEL_COUNT];



/* Helper routine for computing variance from a sum of values and sum of
 * squares.  This calculation is governed by the following formulae:
 *
 *  variance(x) = mean((x - mean(x))^2)
 *              = mean(x^2) - mean(x)^2
 *
 * and so (where mean(x) = N * sum(x))
 *
 *  N * variance(x) = sum(x^2) - sum(x)^2 / N
 *
 * which is what is returned by the calculation below. */

static REAL variance(int sum_values, long long int sum_squares, int samples)
{
    return (REAL) sum_squares - (REAL) sum_values * sum_values / samples;
}


/* Helper routine for writing real values to ai fields. */

static int aiValue(REAL x)
{
    return (int) round(AI_SCALE * x);
}


/* Given a base angle (in the range (-180..180]*AI_SCALE) and a complex
 * number x, returns the relative phase of x, also reduced to the same
 * range. */

static int aiPhase(const complex x, int BaseAngle=0)
{
    const int HALF_TURN = (int) AI_SCALE * 180;
    int Angle = (int) round(HALF_TURN / M_PI * arg(x)) - BaseAngle;
    if (Angle <= - HALF_TURN)
        Angle += 2 * HALF_TURN;
    else if (Angle > HALF_TURN)
        Angle -= 2 * HALF_TURN;
    return Angle;
}



/* All demultiplexing arrays are configured with simple permutation reversing
 * matrices.  There is an aspiration to do crosstalk correction here, but the
 * obstacles are considerable. */

static void InitialiseDemuxArray()
{
    for (int sw = 0; sw < SWITCH_COUNT; sw ++)
    {
        const PERMUTATION & p = PermutationLookup[sw];
        DEMUX_ARRAY Demux;
        memset(Demux, 0, sizeof(DEMUX_ARRAY));
        for (int b = 0; b < BUTTON_COUNT; b ++)
            /* The size of the units here determine the number of bits
             * downstream available for further signal processing.  To ensure
             * no potential loss of bits here we assign the maximum possible
             * value, 2^17. */
            Demux[b][p[b]] = 1 << 17;
        WriteDemuxArray(sw, Demux);
    }
}



/*****************************************************************************/
/*                                                                           */
/*                    Central Signal Conditioning Thread                     */
/*                                                                           */
/*****************************************************************************/


/* The signal conditioning thread runs periodically to manage the state of
 * the correction matrices in the digital signal conditioning (DSC) part of
 * the FPGA.
 *    The button signals received by Libera undergo the following stages of
 * processing
 *
 *  1. Cross bar switching of button inputs: each of four RF channels is
 *     selected to process each of the button inputs.  After a complete round
 *     of switching, all button inputs are processed through all channels.
 *     
 *  2. RF channel processing: controlled amplification and attenuation
 *     together with narrow band filtering.
 *     
 *  3. ADC conversion (this feed is measured directly by FT processing).
 *  
 *  5. Amplitude and phase compensation on the raw ADC readings: this is done
 *     by separate two tap filters on each of the four demultiplexed button
 *     inputs, with a separate set of filters defined for each channel.
 *
 *  4. Demultiplexing and crosstalk compensation: this is done by separate 4x4
 *     matrices, one per switch position, computing the final sampled stream
 *     from the demultiplexed button inputs.
 *
 */

class CONDITIONING : public LOCKED_THREAD
{
public:
    CONDITIONING(REAL f_if) :
        LOCKED_THREAD("Conditioning"),
        
        cotan_if(1.0/tan(f_if)),
        cosec_if(1.0/sin(f_if)),
        m_cis_if(exp(-I*f_if)),
        IqData(SAMPLE_SIZE, true),
        IqDigestWaveform(MAX_SWITCH_SEQUENCE * BUTTON_COUNT)
    {
        /* Establish defaults for configuration variables before reading
         * their currently configured values. */
        MaximumDeviationThreshold = aiValue(2.);    // Default = 2%
        ChannelIIRFactor = aiValue(0.1);
        ConditioningInterval = 1000;                // 1 s

        Persistent("SC:MAXDEV",   MaximumDeviationThreshold);
        Persistent("SC:CIIR",     ChannelIIRFactor);
        Persistent("SC:INTERVAL", ConditioningInterval);


        /* Initialise the switches and demultiplexing matrices. */
        Publish_ao("SC:MAXDEV",   MaximumDeviationThreshold);
        Publish_ao("SC:CIIR",     ChannelIIRFactor);
        Publish_ao("SC:INTERVAL", ConditioningInterval);

        /* Initialise state. */
        ConditioningStatus = SC_OFF;
        /* Ensure we start with fresh channel values on startup! */
        ResetChannelIIR = true;

        /* Publish PVs. */
        Publish_mbbi("SC:STATUS", ConditioningStatus);
        Publish_ai("SC:DEV", Deviation);
        Publish_ai("SC:PHASEB", PhaseB);
        Publish_ai("SC:PHASEC", PhaseC);
        Publish_ai("SC:PHASED", PhaseD);
        
        IqData.Publish("SC");
        Publish_waveform("SC:IQDIGEST", IqDigestWaveform);

        for (int c = 0; c < CHANNEL_COUNT; c ++)
        {
            char Channel[10];
            sprintf(Channel, "SC:C%d", c + 1);
            Publish_ai(Concat(Channel, "PHASE"), ChannelPhase[c]);
            Publish_ai(Concat(Channel, "MAG"),   ChannelMag[c]);
            Publish_longin(Concat(Channel, "RAW0"), PhaseArray[c][0]);
            Publish_longin(Concat(Channel, "RAW1"), PhaseArray[c][1]);
        }
        Publish_ai("SC:CSCALE", ChannelScale);

        Interlock.Publish("SC");
    }

    /* This controls whether signal conditioning is operational. */
    void EnableConditioning(bool SetEnabled)
    {
        Enabled = SetEnabled;
    }

    /* Changing attenuation is synchronised with condition processing. */
    bool SetAttenuation(int NewAttenuation)
    {
        Lock();
        bool Ok = WriteAttenuation(NewAttenuation)  &&  CommitDscState();
        ResetChannelIIR = true;
        Unlock();
        return Ok;
    }
    
    
    
    void SetUnityCompensation()
    {
        COMPENSATION_MATRIX Compensation;
        for (int c = 0; c < CHANNEL_COUNT; c ++)
            Compensation[c] = 1.0;
        WritePhaseCompensation(Compensation);
    }


private:

    bool WritePhaseCompensation(const COMPENSATION_MATRIX& Compensation)
    {
        PHASE_ARRAY NewPhaseArray;
        COMPENSATION_MATRIX NewActual;
        bool Ok = true;
        for (int c = 0; Ok  &&  c < CHANNEL_COUNT; c ++)
            Ok = ComplexToTwoPole(
                Compensation[c], NewPhaseArray[c], NewActual[c]);

        /* Only actually write the phase compensation if there was no
         * overflow in the conversion. */
        if (Ok)
        {
            for (int sw = 0; sw < SWITCH_COUNT; sw ++)
                WritePhaseArray(sw, NewPhaseArray);
            memcpy(ActualCompensation, NewActual, sizeof(NewActual));
            memcpy(PhaseArray, NewPhaseArray, sizeof(PHASE_ARRAY));
        }
        return Ok;
    }


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /*                         Signal Processing Core                        */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


    typedef complex IQ_DIGEST[MAX_SWITCH_SEQUENCE][BUTTON_COUNT];

    enum SC_STATE
    {
        SC_OFF,
        SC_NO_DATA,
        SC_NO_SWITCH,
        SC_VARIANCE,
        SC_OVERFLOW,
        SC_OK
    };

    /* This routine computes the appropriate form of phase and magnitude
     * compensation term to be written to the FPGA.  Internally each
     * compensation is expressed as a complex number representing the desired
     * phase and amplitude correction, but in the FPGA this is implemented as
     * a two pole filter.
     *    We make the conversion on the assumption that we're dealing with a
     * narrow band signal at the machine intermediate frequency.  Then the
     * effect of a two pole filter
     * 
     *                   -1
     *      F = a  + a  z
     *           0    1
     *
     * on an input signal of the form z=exp(i w) -- w is the intermediate
     * frequency in radians per sample -- is to multiply the signal by
     *
     *      F(w) = a  + a  (cos w - i sin w)  .
     *              0    1
     *
     * If we equate this to the desired compensation K=x+iy then we simply
     * need to solve for 
     *
     *      F(w) = x + i y  ,
     *
     * or, in other words
     *
     *                 cos w
     *      a  = x + y ----- = x + y cot w
     *       0         sin w
     *       
     *               y
     *      a  = - ----- = - y csc w
     *       1     sin w
     *
     * After this calculation we recompute the associated complex value so
     * that we can take digitisation errors into account. */
    bool ComplexToTwoPole(const complex xy, PHASE_ENTRY &F, complex &actual)
    {
        F[0] = (int) round(PHASE_UNITY * (real(xy) + imag(xy) * cotan_if));
        F[1] = (int) round(PHASE_UNITY * (- imag(xy) * cosec_if));

        /* Now compute what's actually been written taking into account the
         * limited 18 bit precision. */
        int shift = 32 - 18;    // 32 bit word, 18 significant bits (+ sign)
        int F0 = (F[0] << shift) >> shift;
        int F1 = (F[1] << shift) >> shift;
        actual = ((REAL) F0 + m_cis_if * (REAL) F1) / (REAL) PHASE_UNITY;

        /* Overflow detection. */
        bool Ok = F0 == F[0]  &&  F1 == F[1];
        if (!Ok)
            printf("Overflow converting %f + %f i to [%d, %d]\n",
                real(xy), imag(xy), F[0], F[1]);
        return Ok;
    }
    

    /* Signal conditioning reading needs to run concurrently with existing
     * data capture, so to avoid interference we need to open a separate
     * device handle. */
    bool ReadWaveform(LIBERA_ROW *Data, size_t Length)
    {
        int TargetLength = Length * sizeof(LIBERA_ROW);
        int Read;
        return
            TEST_IO(Read, "Error seeking", lseek, DevDd, 0, CSPI_SEEK_ST)  &&
            TEST_IO(Read, "Error reading",
                read, DevDd, Data, TargetLength)  &&
            TEST_OK(Read == TargetLength, "Read from DD was incomplete");
    }

    
    /* Searches for the start of the next switching marker in the waveform.
     * This is signalled by the bottom bit of the I data. */
    bool SwitchMarker(const LIBERA_ROW *Data, size_t Length, size_t &Marker)
    {
        /* First make sure we skip past any marker that happens at the start
         * of our data block. */
        while (Marker < Length  &&  (Data[Marker][0] & 1) == 1)
            Marker ++;
        /* Now skip to the next marker. */
        while (Marker < Length  &&  (Data[Marker][0] & 1) == 0)
            Marker ++;
        /* Either we're there or we've run out of buffer. */
        return Marker < Length;
    }


    /* This routine extracts the button readings for each switch position,
     * producing an array IqDigest with
     *      IqDigest[ix][c] = average reading for button c for switch ix.
     * The button positions are reduced to complex numbers scaled by the
     * overall average reading.  The variance of the data is also computed
     * for thresholding further work. */
    bool DigestWaveform(const LIBERA_ROW *Data, IQ_DIGEST &IqDigest)
    {
        int Totals[MAX_SWITCH_SEQUENCE][2*BUTTON_COUNT];
        long long int Squares[MAX_SWITCH_SEQUENCE][2*BUTTON_COUNT];
        memset(Totals, 0, sizeof(Totals));
        memset(Squares, 0, sizeof(Squares));

        /* Work through all full switch cycles in the captured waveform
         * accumulating total readings by button and switch position.  Also
         * accumulate squares so we can compute the variance at the end for
         * sanity checking. */
        const size_t SampleLength = SWITCH_PERIOD * SwitchSequenceLength;
        size_t Marker = 0;
        int Samples = 0;
        while(SwitchMarker(Data, SAMPLE_SIZE - SampleLength, Marker))
        {
            Samples += SWITCH_PERIOD - SWITCH_HOLDOFF;
            /* Work through each of the switch positions, pushing both the
             * switch index and the marker. */
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            {
                const int Start = Marker + ix * SWITCH_PERIOD;
                /* Skip the first few points after the switch transition, as
                 * the data in this part is a bit rough. */
                for (int i = SWITCH_HOLDOFF; i < SWITCH_PERIOD; i ++)
                {
                    /* Work through all of the I and Q button readings. */
                    for (int b = 0; b < 2*BUTTON_COUNT; b ++)
                    {
                         /* We can accumulate prescaled integer values here
                          * without penalty: the incoming raw ADC data has up
                          * to 16 bits of precision, and subsequent
                          * turn-by-turn filtering adds perhaps 8 more.  We
                          * can therefore prescale by 8 without penalty. */
                        int Value = Data[Start + i][b] >> PRESCALE;
                        Totals[ix][b] += Value;
                        Squares[ix][b] += (long long int) Value * Value;
                    }
                }
            }
        }

        /* If no switch markers seen then can do nothing more. */
        if (Samples == 0)   return false;

        /* Now reduce the raw summed data to averages and overall variance as
         * complex numbers. */
        REAL Variance = 0.0;
        REAL MinimumSignal = LONG_MAX;
        REAL Prescale = (REAL) (1 << PRESCALE); 
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            for (int b = 0; b < BUTTON_COUNT; b ++)
            {
                IqDigest[ix][b] = (Prescale / Samples) * complex(
                    (REAL) Totals[ix][2*b], (REAL) Totals[ix][2*b+1]);
                Variance +=
                    variance(Totals[ix][2*b],   Squares[ix][2*b],   Samples) +
                    variance(Totals[ix][2*b+1], Squares[ix][2*b+1], Samples);
                REAL Signal = abs(IqDigest[ix][b]);
                if (Signal < MinimumSignal)
                    MinimumSignal = Signal;
            }
        }
        Variance /= SwitchSequenceLength * BUTTON_COUNT * Samples;
        Deviation = aiValue(100. * Prescale * sqrt(Variance) / MinimumSignal);

        return true;
    }

    
    /* Given the raw inferred input signals this computes the angles and
     * updates the appropriate fields. */
    void UpdateSignalIn(const complex SignalIn[BUTTON_COUNT])
    {
        int PhaseA = aiPhase(SignalIn[0]);
        PhaseB = aiPhase(SignalIn[1], PhaseA);
        PhaseC = aiPhase(SignalIn[2], PhaseA);
        PhaseD = aiPhase(SignalIn[3], PhaseA);
    }

    /* Given an array of channels updates the appropriate fields. */
    void UpdateChannels(const COMPENSATION_MATRIX &Channels)
    {
        for (int c = 0; c < CHANNEL_COUNT; c ++)
        {
            ChannelPhase[c]  = aiPhase(Channels[c]);
            ChannelMag[c] = aiValue(abs(Channels[c]));
        }
    }


    void ProcessIqDigest(
        const IQ_DIGEST &IqDigest, COMPENSATION_MATRIX &NewCompensation)
    {
        /* Compute an estimate of the incoming signal on each button.  The
         * underlying model is
         *
         *      Y[n,b] = K[p[n,b]] C[p[n,b]] X[b]
         *      
         * where
         *      n = switch position
         *      b = button number
         *      p[n,b] = channel processing button b in switch position n
         *      Y[n,b] = recorded signal for button b in switch position n
         *      K[c] = currently applied correction factor for channel c
         *      C[c] = (modelled) gain of channel c
         *      X[b] = input signal on button b.
         *
         * We estimate
         *
         *      X^[b] = mean_n(Y[n,b] / K[p[n,b]])
         *            = mean_n(C[p[n,b]]) X[b]
         *            = mean_c(C[c]) X[b]
         *            ~~ X[b]
         *
         * This derivation relies on the necessary assumption that p[n,b]
         * covers all channels c, and we have to assume mean(C) = 1. 
         *
         * It helps to first compute uncorrected channel outputs, let's write
         * them
         *
         *      Z[n,p[n,b]] = Y[n,b] / K[p[n,b]]
         * or
         *      Z[n,c] = Y[n,q[n,c]] / K[c]
         *
         * where
         *      c = channel number = p[n,b] 
         *      p[n,q[n,c]] = c
         *      q[n,q[n,b]] = b
         *
         * and then
         *      Z[n,c] = C[c] X[q[n,c]]
         *      X^[b] = mean_n(Z[n,b])
         */
        IQ_DIGEST RawChannels;
        complex SignalIn[BUTTON_COUNT];
        for (int b = 0; b < BUTTON_COUNT; b ++)
            SignalIn[b] = 0.0;
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            const PERMUTATION &p = PermutationLookup[(int)SwitchSequence[ix]];
            for (int b = 0; b < BUTTON_COUNT; b ++)
            {
                complex RawSignal = IqDigest[ix][b] / ActualCompensation[p[b]];
                RawChannels[ix][p[b]] = RawSignal;
                SignalIn[b] += RawSignal;
            }
        }
        for (int b = 0; b < BUTTON_COUNT; b ++)
            SignalIn[b] /= (REAL) SwitchSequenceLength;
        /* Publish the measured signal angles. */
        UpdateSignalIn(SignalIn);
            
        /* Now we want to compute a new value for the correction factor, let's
         * call it K', to ensure that K=1/C.  However, we'll first need to
         * compute (an estimate for ) C directly, so for each n we can compute
         *
         *      C^[p[n,b]] = Y[n,b] / X[b] K[p[n,b]]
         *                 = Z[n,p[n,b]] / X[b] 
         *
         * and we'll take C^[c] as the mean of these values. */
        COMPENSATION_MATRIX NewChannels;
        for (int c = 0; c < CHANNEL_COUNT; c ++)
            NewChannels[c] = 0.0;
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            const PERMUTATION &p = PermutationLookup[(int)SwitchSequence[ix]];
            for (int b = 0; b < BUTTON_COUNT; b ++)
                NewChannels[p[b]] += RawChannels[ix][p[b]] / SignalIn[b];
        }
        for (int c = 0; c < CHANNEL_COUNT; c ++)
            NewChannels[c] /= (REAL) SwitchSequenceLength;

        /* Now either merge the new channel values into the existing channel
         * value using our current IIR value, or simply assign the new
         * values. */
        if (ResetChannelIIR)
        {
            ResetChannelIIR = false;
            memcpy(CurrentChannels, NewChannels, sizeof(NewChannels));
        }
        else
        {
            REAL IIR = (REAL) ChannelIIRFactor / AI_SCALE;
            for (int c = 0; c < CHANNEL_COUNT; c ++)
                CurrentChannels[c] =
                    (1 - IIR) * CurrentChannels[c] + IIR * NewChannels[c];
        }
        /* Published the updated channel values. */
        UpdateChannels(CurrentChannels);

        /* Finally there is a slightly odd correction we need to make.  When
         * operating without signal correction the default state is to set
         * the channel compensation matrix to unity, and we would like the
         * overall response of the system to change as little as possible
         * when compensation is enabled.
         *    However, there is a complication: after the data has been
         * reduced to turn-by-turn (as processed here) the phase information
         * is then taken away by taking magnitudes, and effectively the data
         * is then averages over switches.  This means, in effect, that the
         * FF and SA streams see
         * 
         *      X^[b] = mean_n(|C[p[n,b]] X[b]|)
         *            = mean_c(|C[c]|) X[b]  .
         *
         * It's an unavoidable fact that mean_c(|C[c]|) >= |mean_c(C[c])|
         * with equality only when they're all in phase.  As our correction
         * process effectively cancels out all the C[c] terms, we end up
         * reducing the data intensity by
         *
         *      S = mean_c(|C[c]|)
         *
         * so here we compute this so that we can put it back into the final
         * compensation matrix with values S/C^[c]. */
        REAL MagnitudeScaling = 0.0;
        for (int c = 0; c < CHANNEL_COUNT; c ++)
            MagnitudeScaling += abs(CurrentChannels[c]);
        MagnitudeScaling /= CHANNEL_COUNT;
        ChannelScale = aiValue(MagnitudeScaling);
        
        for (int c = 0; c < CHANNEL_COUNT; c ++)
            NewCompensation[c] = MagnitudeScaling / CurrentChannels[c];
    }


    /* Processes a single round of signal conditioning: reads a waveform,
     * extracts switch dependent button readings, and computes the
     * compensation matrix. */
    SC_STATE ProcessSignalConditioning()
    {
        LIBERA_ROW * Waveform = (LIBERA_ROW *) IqData.Waveform();
        if (!ReadWaveform(Waveform, SAMPLE_SIZE))
            return SC_NO_DATA;

        /* Capture one waveform and extract the raw switch/button matrix. */
        IQ_DIGEST & IqDigest = * (IQ_DIGEST *) IqDigestWaveform.Array();
        if (!DigestWaveform(Waveform, IqDigest))
            return SC_NO_SWITCH;

        /* Check the signal deviation: if it's too high, don't try anything
         * further. */
        if (Deviation > MaximumDeviationThreshold)
            return SC_VARIANCE;

        /* Compute the new updated compensation matrix. */
        COMPENSATION_MATRIX NewCompensation;
        ProcessIqDigest(IqDigest, NewCompensation);
        
        /* If writing into the FPGA overflows then bail out. */
        if (!WritePhaseCompensation(NewCompensation))
            return SC_OVERFLOW;

        /* All done: a complete cycle. */
        CommitDscState();
        return SC_OK;
    }

    
    void Thread()
    {
        SetUnityCompensation();
        
        if (!TEST_IO(DevDd, "Unable to open /dev/libera.dd for conditioning",
                open, "/dev/libera.dd", O_RDONLY))
            /* Returning early causes error return. */
            return;
        StartupOk();

        while(Running())
        {
//            sleep(1);
            Lock();
            WaitFor(ConditioningInterval);
            Unlock();
            
            Interlock.Wait();
            
            Lock();
            if (Enabled)
                ConditioningStatus = ProcessSignalConditioning();
            else
                ConditioningStatus = SC_OFF;
            Unlock();
            
            Interlock.Ready();
        }
    }



    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /*                      Conditioning Thread Variables                    */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    
    /* A handful of constants derived from the machine intermediate frequency
     * and used for phase compensation calculation. */
    const REAL cotan_if;    // cotangent of IF
    const REAL cosec_if;    // cosecant of IF
    const complex m_cis_if; // -exp(i * IF)

    int DevDd;

    /* This flag controls whether signal conditioning is operational. */
    bool Enabled;
    /* This controls (in milliseconds) the interval between conditioning
     * rounds. */
    int ConditioningInterval;

    /* Variables exposed through epics. */
    int ConditioningStatus;
    int MaximumDeviationThreshold;
    int Deviation;

    /* Raw IQ waveform as read. */
    IQ_WAVEFORMS IqData;
    /* Digested IQ data. */
    COMPLEX_WAVEFORM IqDigestWaveform;

    /* Button phases, all relative to button A. */
    int PhaseB, PhaseC, PhaseD;
    /* Channel readings. */
    int ChannelPhase[CHANNEL_COUNT];
    int ChannelMag[CHANNEL_COUNT];
    int ChannelScale;
    /* Actual phase compensation array as written. */
    PHASE_ARRAY PhaseArray;

    bool ResetChannelIIR;
    int ChannelIIRFactor;

    INTERLOCK Interlock;

    /* This is the compensation matrix as actually written to the FPGA: we
     * need this in order to accurately reverse its effects! */
    COMPENSATION_MATRIX ActualCompensation;

    /* This is the array of channel gains as measured. */
    COMPENSATION_MATRIX CurrentChannels;
};



static CONDITIONING * ConditioningThread = NULL;




/*****************************************************************************/
/*                                                                           */
/*                        External Interface Routines                        */
/*                                                                           */
/*****************************************************************************/





bool WriteDscMode(CSPI_DSCMODE DscMode)
{
    switch (DscMode)
    {
        case CSPI_DSC_OFF:
            ConditioningThread->EnableConditioning(false);
            break;
        case CSPI_DSC_AUTO:
            ConditioningThread->EnableConditioning(true);
            break;
        case CSPI_DSC_UNITY:
            ConditioningThread->EnableConditioning(false);
            ConditioningThread->SetUnityCompensation();
            CommitDscState();
            break;
        default:
            printf("Unexpected DSC mode %d\n", DscMode);
            break;
    }
    return true;
}


bool WriteSwitchState(CSPI_SWITCHMODE Switches)
{
    AutoSwitchState = Switches == CSPI_SWITCH_AUTO;
    bool Ok;
    if (AutoSwitchState)
        Ok = WriteSwitchSequence(SwitchSequence, SwitchSequenceLength);
    else
    {
        ManualSwitch = Switches;
        Ok = WriteSwitchSequence((char *)&Switches, 1);
    }
    return Ok  &&  CommitDscState();
}


const PERMUTATION & SwitchPermutation()
{
    return PermutationLookup[ManualSwitch];
}


bool WriteDscAttenuation(int Attenuation)
{
    return ConditioningThread->SetAttenuation(Attenuation);
}






bool InitialiseSignalConditioning(int Harmonic, int Decimation)
{
    /* Select the appropriate switches and initialise the demultiplexor
     * array. */
    if (Brilliance())
    {
        SwitchSequence = BrillianceSwitchSequence;
        SwitchSequenceLength = ARRAY_SIZE(BrillianceSwitchSequence);
    }
    else
    {
        SwitchSequence = ElectronSwitchSequence;
        SwitchSequenceLength = ARRAY_SIZE(ElectronSwitchSequence);
    }
    InitialiseDemuxArray();
    
    /* Start the conditioning thread.  The intermediate frequency needs to be
     * in radians per sample. */
    REAL f_if = 2 * M_PI * (REAL) (Harmonic % Decimation) / Decimation;
    ConditioningThread = new CONDITIONING(f_if);
    ConditioningThread->StartThread();
    
    CommitDscState();


    return true;
}

void TerminateSignalConditioning()
{
    if (ConditioningThread != NULL)
        ConditioningThread->Terminate();
}
