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
#include "interlock.h"

#include "conditioning.h"



/* There are two standard switch sequences that we use: an 8 round sequence
 * for Libera Electron, and a 4 round sequence for Libera Brilliance. */
static const char ElectronSwitchSequence[8]   = { 3, 7, 15, 11, 0, 4, 12, 8 };
static const char BrillianceSwitchSequence[4] = { 15, 0, 9, 6 };


/* The arrays below translate switch positions into button permutations.  This
 * is needed when reading raw ADC buffers to undo the permutation performed by
 * the input switch, and is also needed during signal conditioning processing
 * to correlate readings with channels.  For each permutuation row the entry
 * p[b] determines which ADC channel is processing the signal for button b.
 *
 * Oddly enough, Libera Brilliance uses a different configuration of switches
 * from Libera Electron, so we need a completely different permutation lookup
 * table to handle this!
 *
 *
 * The array of switches can be understood to be implemented as an array of
 * four binary cross-bar switches where each individual switch
 *
 *        +----+
 *    a --+ s  +-- c   is either connected straight through, a-c and b-d 
 *    b --+  i +-- d   (when s_i=0) or crossed over (a-d, b-c, s_i=1).
 *        +----+
 *
 *   Button                       Channel     Bit sequence:
 *    E  B                         E  B
 *           +----+       +----+              
 *    A  D --+ s  +-------+ s  +-- 3  1       Electron:
 *    B  A --+  0 +--   --+  2 +-- 0  0           s  s  s  s
 *           +----+  \ /  +----+                   0  1  2  3
 *                    X
 *           +----+  / \  +----+
 *    D  C --+ s  +--   --+ s  +-- 2  2       Brilliance:
 *    C  B --+  1 +-------+  3 +-- 1  3           s  s  s  s
 *           +----+       +----+                   2  0  3  1
 *
 * This figure shows the switch topology, how it is connected for each set of
 * buttons and channels (where the channel identifiers correspond to indexes
 * into FPGA structures) and the mapping of switches to bits in the switch
 * selector.  This results in the permutations tabulated below. */

static const PERMUTATION ElectronPermutationLookup[] =
{
    { 3, 2, 1, 0 },  { 3, 1, 2, 0 },  { 0, 2, 1, 3 },  { 0, 1, 2, 3 },
    { 3, 2, 0, 1 },  { 3, 1, 0, 2 },  { 0, 2, 3, 1 },  { 0, 1, 3, 2 },
    { 2, 3, 1, 0 },  { 1, 3, 2, 0 },  { 2, 0, 1, 3 },  { 1, 0, 2, 3 },
    { 2, 3, 0, 1 },  { 1, 3, 0, 2 },  { 2, 0, 3, 1 },  { 1, 0, 3, 2 }
};

static const PERMUTATION BrillancePermutationLookup[] =
{
    { 2, 3, 0, 1 },  { 2, 0, 3, 1 },  { 3, 2, 0, 1 },  { 3, 0, 2, 1 },
    { 2, 3, 1, 0 },  { 2, 1, 3, 0 },  { 3, 2, 1, 0 },  { 3, 1, 2, 0 },
    { 1, 3, 0, 2 },  { 1, 0, 3, 2 },  { 1, 2, 0, 3 },  { 1, 0, 2, 3 },
    { 0, 3, 1, 2 },  { 0, 1, 3, 2 },  { 0, 2, 1, 3 },  { 0, 1, 2, 3 }
};


/* Some magic numbers to be configurable real soon now. */
#define SWITCH_PERIOD   40
#define SWITCH_HOLDOFF  6
#define SAMPLE_SIZE     2048
#define PRESCALE        8

#define AI_SCALE        1e6




/* This is the currently programmed sequence of switches. */
static const char * SwitchSequence;
static int SwitchSequenceLength;

static const PERMUTATION * PermutationLookup;



/*****************************************************************************/
/*                                                                           */
/*                       Miscellaneous Helper Routines                       */
/*                                                                           */
/*****************************************************************************/

/* The following routines are all part of the COMPENSATION thread below, but
 * don't need to be declared as methods of the thread class. */


/* For some stupid reason gcc has decided that it is "wrong" to assign arrays
 * and that the only way to do this properly is by memcpy.  If you insist...
 * this little hack ensures types match and the right size is used. */
template<class T>
void AssignArray(T &out, const T &in)
{
    memcpy(&out, &in, sizeof(T));
}

template<class T>
void ZeroArray(T &out)
{
    memset(&out, 0, sizeof(T));
}

template<class T> T sqr(const T x) { return x*x; }




/* Helper routine for computing variance from a sum of values and sum of
 * squares.  This calculation is governed by the following formulae:
 *
 *  variance(x) = mean((x - mean(x))^2)
 *              = mean(x^2) - mean(x)^2
 *
 * and so (where mean(x) = N * sum(x))
 *
 *  N^2 * variance(x) = N * sum(x^2) - sum(x)^2
 *
 * which is what is returned by the calculation below. */

static REAL variance(int sum_values, long long int sum_squares, int samples)
{
    return (REAL) (
        samples * sum_squares - sqr((long long int) sum_values));
}


/* Helper routine for writing real values to ai fields. */

static int aiValue(REAL x)
{
    return (int) round(AI_SCALE * x);
}


/* Given an angle x in radians and an optional base angle in ai scaling units
 * returns x scaled to EPICS scaling units and reduced to the range
 *      [-180..180) * AI_SCALE. */

static int aiPhase(const REAL x, int BaseAngle=0)
{
    const int HALF_TURN = (int) AI_SCALE * 180;
    const int FULL_TURN = 2 * HALF_TURN;
    int Angle = (int) round(x * HALF_TURN / M_PI) - BaseAngle;

    /* The following calculation is intended to reduce Angle to the range
     * [-HALF_TURN..HALF_TURN).  If the C % operator was defined properly
     * then this would simply be
     *      (Angle + HALF_TURN) % FULL_TURN - HALF_TURN .
     * Unfortunately negative values behave unpredictably, so I have to
     * handle that specially. */
    return (Angle + HALF_TURN + FULL_TURN * (1 + Angle / FULL_TURN)) %
        FULL_TURN - HALF_TURN;
}



/* All demultiplexing arrays are configured with simple permutation reversing
 * matrices.  There is an aspiration to do crosstalk correction here, but the
 * obstacles are considerable. */

static void NormalDemuxArray()
{
    for (int sw = 0; sw < SWITCH_COUNT; sw ++)
    {
        const PERMUTATION & p = PermutationLookup[sw];
        DEMUX_ARRAY Demux;
        ZeroArray(Demux);
        for (int b = 0; b < BUTTON_COUNT; b ++)
            /* The size of the units here determine the number of bits
             * downstream available for further signal processing.  To ensure
             * no potential loss of bits here we assign the maximum possible
             * value, 2^17. */
            Demux[b][p[b]] = 1 << 17;
        WriteDemuxArray(sw, Demux);
    }
}


/* For testing the demultiplexing array can be disabled.  This is done by
 * writing an identity matrix into all of the switch positions. */

static void TrivialDemuxArray()
{
    DEMUX_ARRAY Demux;
    ZeroArray(Demux);
    for (int b = 0; b < BUTTON_COUNT; b ++)
        Demux[b][b] = 1 << 17;
    for (int sw = 0; sw < SWITCH_COUNT; sw ++)
        WriteDemuxArray(sw, Demux);
}


/* Searches for the start of the next switching marker in the waveform.
 * This is signalled by the bottom bit of the I data. */

static bool SwitchMarker(const LIBERA_ROW *Data, size_t Length, size_t &Marker)
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
        EpicsWritePhaseArray(*this)
    {
        /* Establish defaults for configuration variables before reading
         * their currently configured values. */
        MaximumDeviationThreshold = aiValue(2.);    // Default = 2%
        ChannelIIRFactor = aiValue(0.1);
        ConditioningInterval = 5000;                // 5 s

        /* Initialise state. */
        ConditioningStatus = SC_OFF;
        /* Ensure we start with fresh channel values on startup! */
        ResetChannelIIR = true;

        /* Configuration parameters: maximum acceptable deviation when
         * processing, IIR factor and how frequently we run. */
        Persistent("SC:MAXDEV",   MaximumDeviationThreshold);
        Persistent("SC:CIIR",     ChannelIIRFactor);
        Persistent("SC:INTERVAL", ConditioningInterval);
        Publish_ao("SC:MAXDEV",   MaximumDeviationThreshold);
        Publish_ao("SC:CIIR",     ChannelIIRFactor);
        Publish_ao("SC:INTERVAL", ConditioningInterval);

        /* General conditioning status PV.  The alarm state of this can
         * usefully be integrated into the overall system health. */
        Publish_mbbi("SC:STATUS", ConditioningStatus);
        /* More detailed PVs for information about the state of conditioning.
         *  DEV     Relative standard deviation of last set of readings
         *  PHASEB,C,D  Relative phases of inputs on the four buttons (all
         *          relative to the phase of button A. */
        Publish_ai("SC:DEV", Deviation);
        Publish_ai("SC:PHASEB", PhaseB);
        Publish_ai("SC:PHASEC", PhaseC);
        Publish_ai("SC:PHASED", PhaseD);

        /* The raw IQ waveform used for SC processing is made available, as
         * are some of the intermediate stages of processing. */
        IqData.Publish("SC");
        PublishSimpleWaveform(complex, "SC:IQDIGEST", IqDigest);
        PublishSimpleWaveform(int,     "SC:LASTCOMP", OldPhaseArray);
        PublishSimpleWaveform(int,     "SC:COMP", CurrentPhaseArray);
        /* This waveform allows the compensation matrix to be written
         * directly: only for research and testing use! */
        Publish_waveform("SC:SETCOMP_S", EpicsWritePhaseArray);

        for (int c = 0; c < CHANNEL_COUNT; c ++)
        {
            char Channel[10];
            sprintf(Channel, "SC:C%d", c + 1);
            /* For each ADC channel we publish the measured phase and
             * magnitude together with the corresponding raw FIR coefficients
             * used for channel compensation. */
            Publish_ai(Concat(Channel, "PHASE"), ChannelPhase[c]);
            Publish_ai(Concat(Channel, "MAG"),   ChannelMag[c]);
            Publish_ai(Concat(Channel, "VAR"),   ChannelVariance[c]);
        }

        Interlock.Publish("SC");
    }


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /*                      Externally Published Methods                     */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /* For laziness we use one large lock for everything.  All SC processing
     * is done inside the lock, and any related changes to the FPGA
     * (attenuators, switches or matrices) must also be done inside the lock:
     * in particular, CommitDscState() must only be called inside the lock.
     *     Note that holding a lock for a long time is generally very bad
     * practice, and the best way to handle this would be to convert these
     * methods into commands to the thread which are processed in a single
     * place: then locking would only be required to add commands to the
     * queue.
     *     However, in this case we really don't care! */

    
    /* Writes the selected switch sequence. */
    bool LockedWriteSwitches(const SWITCH_SEQUENCE Switches, size_t Length)
    {
        Lock();
        bool Ok =
            WriteSwitchSequence(Switches, Length)  &&
            CommitDscState();
        Unlock();
        return Ok;
    }


    /* Controls the state of the conditioning thread and the associated
     * configured compensation matrices. */
    void WriteScMode(SC_MODE ScMode)
    {
        Lock();
        switch (ScMode)
        {
            case SC_MODE_AUTO:
                /* If we've just enabled auto mode then trigger a round of
                 * processing immediately. */
                if (!Enabled)
                    Signal();
                Enabled = true;
                break;
                
            case SC_MODE_UNITY:
                /* Special processing for switching into UNITY mode: in this
                 * case we revert the compensation matrices.  As we're
                 * changing the state, we hold off the interlock.  Ensure we
                 * start from scratch when reenabling. */
                ResetChannelIIR = true;
                SetUnityCompensation();
                HoldoffInterlock();
                CommitDscState();
                
                Enabled = false;
                break;

            case SC_MODE_FIXED:
                /* Use the last good compensation matrix in this mode.
                 * Again, as we're (potentially) making a glitch, request an
                 * interlock holdoff. */
                WritePhaseCompensation(CurrentCompensation);
                HoldoffInterlock();
                CommitDscState();
                
                Enabled = false;
                break;
        }
        Unlock();
    }


    /* Changing attenuation is synchronised with condition processing.  We
     * trigger an immediate round of processing. */
    bool ScWriteAttenuation(int NewAttenuation)
    {
        Lock();
        /* The interlock must be temporarily disabled before changing the
         * attenuation. */
        HoldoffInterlock();
        bool Ok =
            WriteAttenuation(NewAttenuation)  &&
            CommitDscState();
        if (Ok)
        {
            ResetChannelIIR = true;
            Signal();
        }
        Unlock();
        return Ok;
    }
    
    
private:
    /* The following basic datatypes are used for processing.  We distinguish
     * the two arrays BUTTONS and CHANNELS as an important discipline for
     * keeping track, though in fact one is just a permutation of the other. */
    typedef complex BUTTONS[BUTTON_COUNT];
    typedef complex CHANNELS[CHANNEL_COUNT];
    typedef BUTTONS BUTTON_ARRAY[MAX_SWITCH_SEQUENCE];
    typedef CHANNELS CHANNEL_ARRAY[MAX_SWITCH_SEQUENCE];

    typedef REAL BUTTONS_REAL[BUTTON_COUNT];

    /* This is used to record the actual phase array written in each switch
     * position. */
    typedef PHASE_ARRAY PHASE_ARRAY_LIST[MAX_SWITCH_SEQUENCE];


    /* Conditioning state as reported through SC:STATUS. */
    enum SC_STATE
    {
        SC_OFF,         // SC currently disabled
        SC_NO_DATA,     // Unable to read IQ data: serious problem
        SC_NO_SWITCH,   // No switch marker seen: switches not running?
        SC_VARIANCE,    // Variance in data too large to process
        SC_OVERFLOW,    // Channel compensations too large to write
        SC_OK           // SC working normally
    };

    

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /*                       Phase Compensation Matrices                     */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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
     * This calculation is then check for digitisation errors to avoid
     * writing an invalid value into the FPGA. */
    bool ComplexToTwoPole(const complex xy, PHASE_ENTRY &F)
    {
        F[0] = (int) round(PHASE_UNITY * (real(xy) + imag(xy) * cotan_if));
        F[1] = (int) round(PHASE_UNITY * (- imag(xy) * cosec_if));

        /* Now check that we've actually written what we meant to: has there
         * been any overflow? */
        int shift = 32 - 18;    // 32 bit word, 18 significant bits (+ sign)
        int F0 = (F[0] << shift) >> shift;
        int F1 = (F[1] << shift) >> shift;
        bool Ok = F0 == F[0]  &&  F1 == F[1];
        if (!Ok)
            printf("Integer overflow converting %f + %f i to [%d, %d]\n",
                real(xy), imag(xy), F[0], F[1]);
        return Ok;
    }


    /* Reverses the computation of ComplexToTwoPole above. */
    complex TwoPoleToComplex(const PHASE_ENTRY &F)
    {
        return ((REAL) F[0] + m_cis_if * (REAL) F[1]) / (REAL) PHASE_UNITY;
    }


    /* Writes a new compensation matrix with full error checking.  Also
     * ensures that the currently active compensation array is recorded so
     * that we can take this into account when computing new values. */
    bool WritePhaseCompensation(const CHANNEL_ARRAY& Compensation)
    {
        PHASE_ARRAY_LIST NewPhaseArray;
        bool Ok = true;
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            for (int c = 0; Ok  &&  c < CHANNEL_COUNT; c ++)
                Ok = ComplexToTwoPole(
                    Compensation[ix][c], NewPhaseArray[ix][c]);

        /* Only actually write the phase compensation if there was no
         * overflow in the conversion.  If so, keep track of what the current
         * phase array actually is. */
        if (Ok)
        {
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
                WritePhaseArray(SwitchSequence[ix], NewPhaseArray[ix]);
            AssignArray(CurrentPhaseArray, NewPhaseArray);
        }
        return Ok;
    }


    /* Resets compensation to unity.  This has to bypass the
     * WritePhaseCompensation() routine above to ensure that *all* switch
     * positions are written. */
    void SetUnityCompensation()
    {
        /* Prepare the unity phase array: filter {1,0} does the job. */
        PHASE_ARRAY UnityPhaseArray;
        for (int c = 0; c < CHANNEL_COUNT; c ++)
        {
            UnityPhaseArray[c][0] = PHASE_UNITY;
            UnityPhaseArray[c][1] = 0;
        }
        /* Write this phase array into *all* switches. */
        for (int sw = 0; sw < SWITCH_COUNT; sw ++)
            WritePhaseArray(sw, UnityPhaseArray);
        /* Finally record this as being used for all positions in the switch
         * sequence. */
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            AssignArray(CurrentPhaseArray[ix], UnityPhaseArray);
    }


    /* Returns the currently active compensation matrix as complex numbers. */
    void GetActualCompensation(CHANNEL_ARRAY &Compensation)
    {
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            for (int c = 0; c < CHANNEL_COUNT; c ++)
                Compensation[ix][c] =
                    TwoPoleToComplex(CurrentPhaseArray[ix][c]);
    }


    /* This little class is used to implement external control of the
     * compensation matrix.  Intended for debug and research only. */
    class WRITE_PHASE_ARRAY : public I_WAVEFORM
    {
    public:
        WRITE_PHASE_ARRAY(CONDITIONING &Parent) :
            I_WAVEFORM(DBF_LONG),
            Parent(Parent)
        {
        }
        
    private:
        bool process(void *array, size_t max_length, size_t &new_length)
        {
            PHASE_ARRAY * NewPhaseArray = (PHASE_ARRAY *) array;
            if (new_length == 16 * 4 * 2)
            {
                Parent.Lock();
                for (int sw = 0; sw < 16; sw ++)
                    WritePhaseArray(sw, NewPhaseArray[sw]);
                for (int ix = 0; ix < SwitchSequenceLength; ix ++)
                    AssignArray(Parent.CurrentPhaseArray[ix],
                        NewPhaseArray[SwitchSequence[ix]]);
                CommitDscState();
                Parent.Unlock();
            }
            else
                printf("Incorrect size of phase array %d\n", new_length);
            return true;
        }

        CONDITIONING &Parent;
    };
    

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    /*                         Signal Processing Core                        */
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    
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


    /* This routine extracts the button readings for each switch position,
     * producing an array IqDigest with
     *      IqDigest[ix][c] = average reading for button c for switch ix.
     * The button positions are reduced to complex numbers.  The variance of
     * the data is also computed for thresholding further processing. */
    bool DigestWaveform(const LIBERA_ROW *Data, BUTTON_ARRAY &IqDigest)
    {
        int Totals[MAX_SWITCH_SEQUENCE][2*BUTTON_COUNT];
        long long int Squares[MAX_SWITCH_SEQUENCE][2*BUTTON_COUNT];
        ZeroArray(Totals);
        ZeroArray(Squares);

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

        /* Now condense the raw summed data to complex numbers.  Note that no
         * rescaling is required as all further processing will treat these
         * readings as relative values.
         *    However, we do compute a variance which is then reduced to a
         * relative standard deviation: this does need to be scaled and is
         * used to filter out data with too much noise (or, generally, too
         * much phase shift). */
        REAL Variance = 0.0;
        REAL MinimumSignal = LONG_MAX;
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            for (int b = 0; b < BUTTON_COUNT; b ++)
            {
                IqDigest[ix][b] = complex(
                    (REAL) Totals[ix][2*b], (REAL) Totals[ix][2*b+1]);
                Variance +=
                    variance(Totals[ix][2*b],   Squares[ix][2*b],   Samples) +
                    variance(Totals[ix][2*b+1], Squares[ix][2*b+1], Samples);
                /* Note that the signal here is scaled by the total number of
                 * samples just read. */
                REAL Signal = abs(IqDigest[ix][b]);
                if (Signal < MinimumSignal)
                    MinimumSignal = Signal;
            }
        }
        Variance /= SwitchSequenceLength * BUTTON_COUNT;
        if (MinimumSignal < 1.0)  MinimumSignal = 1.0;  // Avoid divide by zero
        Deviation = aiValue(100. * sqrt(Variance) / MinimumSignal);

        return true;
    }

    
    /* Given the raw inferred input signals this computes the angles and
     * updates the appropriate fields. */
    void UpdateSignalIn(const BUTTONS_REAL &Xarg)
    {
        int PhaseA = aiPhase(Xarg[0]);
        PhaseB = aiPhase(Xarg[1], PhaseA);
        PhaseC = aiPhase(Xarg[2], PhaseA);
        PhaseD = aiPhase(Xarg[3], PhaseA);
    }


    /* Updates the published channel information by digesting the given
     * compensation matrix. */
    void UpdateChannels()
    {
        const CHANNEL_ARRAY &K = CurrentCompensation;
        /* We update each of the phase, magnitude and variance fields with a
         * digest of the corresponding column of the K matrix. */
        for (int c = 0; c < CHANNEL_COUNT; c ++)
        {
            complex Mean = 0;
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
                Mean += K[ix][c];
            Mean /= SwitchSequenceLength;
            
            /* The channel value is the reciprocal of the channel
             * compensation, of course, so we take this into account. */
            ChannelPhase[c] = aiPhase(- arg(Mean));
            ChannelMag[c]   = aiValue(1 / abs(Mean));   // !!! Possible 1/0

            /* Now compute the variance.  The variance of compensations will
             * do, and indeed it's more meaningful... */
            REAL Variance = 0;
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
                Variance += sqr(abs(K[ix][c] - Mean));
            Variance /= SwitchSequenceLength;
            ChannelVariance[c] = aiValue(sqrt(Variance));
        }
    }


    /* The estimation of X[b] from Z[n,b] via the model
     *
     *      Z[n,b] ~= C[p[n,b]] X[b]
     *
     * is a crucial step in the computation of compensation K ~= 1/C.  There
     * are two obvious candidates for estimating X: the arithmetic and
     * geometric means along n of Z[n,b].
     *     It turns out that using the arithmetic mean is too susceptible to
     * errors not caught in the model above, whereas using the geometric mean
     * is fine.  However, the geometric mean loses absolute phase -- and it
     * further turns out that the phase errors in the geometric mean are
     * *not* a problem.
     *     Thus we estimate
     *
     *     X = geo_mean(abs(Z), 0) * exp(1j * angle(mean(Z, 0)))
     *
     * The computed angles for X are also returned separately: this provides
     * a useful observation on the inputs.  Note, of course, that
     * angle(mean(X)) == angle(sum(X)) so a redundant division can be avoided
     * below. */
    void EstimateX(const BUTTON_ARRAY &Z, BUTTONS &X, BUTTONS_REAL &Xarg)
    {
        for (int b = 0; b < BUTTON_COUNT; b ++)
        {
            REAL Magnitude = 1.0;
            complex Sum = 0.0;
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            {
                Magnitude *= abs(Z[ix][b]);
                Sum += Z[ix][b];
            }
            /* Compute geometric mean and angle. */
            Magnitude = pow(Magnitude, 1. / SwitchSequenceLength);
            Xarg[b] = arg(Sum);
            /* Finally return the computed X. */
            X[b] = std::polar(Magnitude, Xarg[b]);
        }
    }


    /* The computation below can be described as follows.
     *
     * Inputs:
     *      p[n,b]  - Channel used to process button b for switch n
     *      K[n,c]  - Current compensation for channel c and switch n
     *      Y[n,b]  - Digested reading at switch position n from button b
     *
     * Outputs:
     *      K_[n,c] - New compensation matrix for next round
     *      Xa[b]   - Input angles
     *
     * The calculation here can be described by the following numpy code:
     *
     *  Z = Y / K[np]
     *  X = EstimateX(Z)
     *  K_ = (X / Z)[nq]
     *
     * where np and nq are indexing tricks used to implement the element by
     * element treatment below. */
    void ProcessIqDigest(const BUTTON_ARRAY &Y)
    {
        /* Pick up the compensation in effect when Y was captured. */
        CHANNEL_ARRAY K;
        GetActualCompensation(K);

        /* Compute decompensated readings: Z = Y / K[np] . */
        BUTTON_ARRAY Z;
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            const PERMUTATION &p = PermutationLookup[SwitchSequence[ix]];
            for (int b = 0; b < BUTTON_COUNT; b ++)
                Z[ix][b] = Y[ix][b] / K[ix][p[b]];
        }

        /* Estimate the input X button values and publish the input angles
         * as an external observation. */
        BUTTONS X;
        BUTTONS_REAL Xarg;
        EstimateX(Z, X, Xarg);
        UpdateSignalIn(Xarg);

        /* Compute the final compensation: K_ = (X / Z)[nq] .  This is run
         * through a simple one pole IIR before being written to the FPGA (by
         * our caller) and published to EPICS here. */
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
        {
            const PERMUTATION &p = PermutationLookup[SwitchSequence[ix]];
            for (int b = 0; b < BUTTON_COUNT; b ++)
                K[ix][p[b]] = X[b] / Z[ix][b];
        }
        RunIIR(K);
        UpdateChannels();
    }


    
    /* To avoid rapid changes of outputs we normally run all compensation
     * values through a simple IIR filter.  However this can be reset at any
     * time by setting the ResetChannelIIR flag: this is done whenever the
     * attenuators are changed. */
    void RunIIR(const CHANNEL_ARRAY &NewK)
    {
        if (ResetChannelIIR)
        {
            /* On a reset simply copy over the new values. */
            ResetChannelIIR = false;
            AssignArray(CurrentCompensation, NewK);
            /* On a fresh set of SC corrections trigger an interlock holdoff,
             * as this change may cause a glitch if we're unlucky. */
            HoldoffInterlock();
        }
        else
        {
            /* During normal IIR operation compute
             *
             *  K = (1 - a) * K  +  a * NewK
             *
             * where the scaling factor a = ChannelIIRFactor is programmed
             * through the EPICS interface. */
            REAL IIR = (REAL) ChannelIIRFactor / AI_SCALE;
            for (int ix = 0; ix < SwitchSequenceLength; ix ++)
                for (int c = 0; c < CHANNEL_COUNT; c ++)
                    CurrentCompensation[ix][c] =
                        (1 - IIR) * CurrentCompensation[ix][c] +
                        IIR * NewK[ix][c];
        }
    }


    /* Called on startup and when compensation has dug itself into such a
     * deep hole that the compensation array has overflowed.  We write unity
     * gains into the compensation matrix. */
    void ResetCurrentCompensation()
    {
        for (int ix = 0; ix < SwitchSequenceLength; ix ++)
            for (int c = 0; c < CHANNEL_COUNT; c ++)
                CurrentCompensation[ix][c] = 1.0;
        /* The interlock holdoff seems a little gratuitous: if we're this
         * deep in a hole the chances of holding interlock are nil. */
        HoldoffInterlock();
        ResetChannelIIR = true;
        WritePhaseCompensation(CurrentCompensation);
    }


    /* Processes a single round of signal conditioning: reads a waveform,
     * extracts switch dependent button readings, and computes the
     * compensation matrix. */
    SC_STATE ProcessSignalConditioning()
    {
        /* Grab a copy of the current data.  Also grab a copy of the current
         * phase array for the sake of any outside observers: this means that
         * the IQ data can be decoded according to the phase compensation in
         * effect at the time it was captured. */
        AssignArray(OldPhaseArray, CurrentPhaseArray);
        LIBERA_ROW * Waveform = (LIBERA_ROW *) IqData.Waveform();
        if (!ReadWaveform(Waveform, SAMPLE_SIZE))
            return SC_NO_DATA;

        /* Capture one waveform and extract the raw switch/button matrix. */
        if (!DigestWaveform(Waveform, IqDigest))
            return SC_NO_SWITCH;

        /* Check the signal deviation: if it's too high, don't try anything
         * further. */
        if (Deviation > MaximumDeviationThreshold)
            return SC_VARIANCE;

        /* Compute the new updated compensation matrix.  This updates
         * CurrentCompensation with the new values.*/
        ProcessIqDigest(IqDigest);

        /* At this point we're commited to doing something: if it's really bad
         * then we'll back off to a known point. */
        SC_STATE Result = SC_OK;
        if (!WritePhaseCompensation(CurrentCompensation))
        {
            /* If writing into the FPGA overflows then we're in real trouble.
             * In this case we should reset CurrentCompensation. */
            ResetCurrentCompensation();
            Result = SC_OVERFLOW;
        }
        CommitDscState();
        return Result;
    }

    
    void Thread()
    {
        Lock();
        /* Configure the demultiplexing array so that channels are
         * demultiplexed to their corresponding buttons for each switch
         * position. */
        NormalDemuxArray();
        SetUnityCompensation();
        ResetCurrentCompensation();
        CommitDscState();
        Unlock();
        
        if (!TEST_IO(DevDd, "Unable to open /dev/libera.dd for conditioning",
                open, "/dev/libera.dd", O_RDONLY))
            /* Returning early causes error return. */
            return;
        StartupOk();

        while(Running())
        {
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

    /* Device handle used to read raw IQ waveforms.  Needs to be abstracted
     * into hardware.h at some point. */
    int DevDd;

    /* This flag controls whether signal conditioning is operational. */
    bool Enabled;
    /* This controls (in milliseconds) the interval between conditioning
     * rounds. */
    int ConditioningInterval;

    /* Reports status of the conditioning thread to EPICS.  The value is
     * drawn from SC_STATE. */
    int ConditioningStatus;
    /* Configures the maxium allowable signal deviation for SC processing. */
    int MaximumDeviationThreshold;
    /* Reports the signal deviation for the last waveform as read. */
    int Deviation;

    /* Raw IQ waveform as read. */
    IQ_WAVEFORMS IqData;
    /* Digested IQ data: published to EPICS for diagnostics and research. */
    BUTTON_ARRAY IqDigest;

    /* Button phases, all relative to button A. */
    int PhaseB, PhaseC, PhaseD;
    /* Channel scalings as computed in phase, magnitude and overall scaling. */
    int ChannelPhase[CHANNEL_COUNT];
    int ChannelMag[CHANNEL_COUNT];
    int ChannelVariance[CHANNEL_COUNT];
    
    /* Set to reset the channel IIR: reset on initialisation, when entering
     * UNITY mode and when changing attenuation. */
    bool ResetChannelIIR;
    /* Current IIR factor: 1 means no history (IIR ineffective), smaller
     * values mean longer time constants. */
    int ChannelIIRFactor;

    INTERLOCK Interlock;

    /* This is the array of channel gains after IIR processing.  This is what
     * is written into the FPGA, after conversion to phase array form.
     *    When unity gain mode is selected this matrix is used to record the
     * last good computed compensation. */
    CHANNEL_ARRAY CurrentCompensation;
    /* Currently written phase compensation array as actually written to the
     * FPGA.  This is then reversed to compute the associated compensation
     * matrix when processing the signal. */
    PHASE_ARRAY_LIST CurrentPhaseArray;
    /* Phase compensation array used when reading current waveform: published
     * to EPICS for diagnostics and research. */
    PHASE_ARRAY_LIST OldPhaseArray;
    
    /* Writing to this waveform allows the phase array to be written directly
     * -- obviously for expermentation only! */
    WRITE_PHASE_ARRAY EpicsWritePhaseArray;
};





/*****************************************************************************/
/*                                                                           */
/*                        External Interface Routines                        */
/*                                                                           */
/*****************************************************************************/


static CONDITIONING * ConditioningThread = NULL;


/* We remember the currently selected manual switch so that we can return the
 * appropriate permutation array. */
static int ManualSwitch = 3;


bool WriteSwitchState(bool AutoSwitch, int NewManualSwitch)
{
    if (AutoSwitch)
        return ConditioningThread->LockedWriteSwitches(
            SwitchSequence, SwitchSequenceLength);
    else
    {
        ManualSwitch = NewManualSwitch;
        return ConditioningThread->LockedWriteSwitches(
            (char *)&ManualSwitch, 1);
    }
}


void WriteScMode(SC_MODE ScMode)
{
    ConditioningThread->WriteScMode(ScMode);
}


bool ScWriteAttenuation(int Attenuation)
{
    return ConditioningThread->ScWriteAttenuation(Attenuation);
}


const PERMUTATION & SwitchPermutation()
{
    return PermutationLookup[ManualSwitch];
}




/*****************************************************************************/
/*                                                                           */
/*                        Initialisation (and debug)                         */
/*                                                                           */
/*****************************************************************************/


/* Naughty very low level debugging stuff.  Run this at your peril!
 * (Actually, the peril level is pretty low!) */

static void SCdebug(const iocshArgBuf *args)
{
    char Line[1024];
    while(
        printf("SCdebug> "),
        fgets(Line, sizeof(Line), stdin))
    {
        switch(Line[0])
        {
            case 't':
                TrivialDemuxArray();
                CommitDscState();
                break;
            case 'n':
                NormalDemuxArray();
                CommitDscState();
                break;
            case '?':
                printf("Debugging code: read the source!\n"
                       "<Ctrl-D> to exit\n");
                break;
            default:
                printf("?\n");
                break;
        }
    }
}

static const iocshFuncDef SCdebugFuncDef = { "SCdebug", 0, NULL };


bool InitialiseSignalConditioning(int Harmonic, int Decimation)
{
    iocshRegister(&SCdebugFuncDef, SCdebug);
    
    /* Select the appropriate switches and initialise the demultiplexor
     * array. */
    if (Brilliance())
    {
        SwitchSequence = BrillianceSwitchSequence;
        SwitchSequenceLength = ARRAY_SIZE(BrillianceSwitchSequence);
        PermutationLookup = BrillancePermutationLookup;
    }
    else
    {
        SwitchSequence = ElectronSwitchSequence;
        SwitchSequenceLength = ARRAY_SIZE(ElectronSwitchSequence);
        PermutationLookup = ElectronPermutationLookup;
    }
    
    /* Start the conditioning thread.  The intermediate frequency needs to be
     * in radians per sample. */
    REAL f_if = 2 * M_PI * (REAL) (Harmonic % Decimation) / Decimation;
    ConditioningThread = new CONDITIONING(f_if);
    return ConditioningThread->StartThread();
}


void TerminateSignalConditioning()
{
    if (ConditioningThread != NULL)
        ConditioningThread->Terminate();
}
