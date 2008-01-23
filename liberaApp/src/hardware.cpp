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


/* Libera device interface: direct access to device drivers. */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/unistd.h>
#include <sys/mman.h>

/* If RAW_REGISTER is defined then raw register access through /dev/mem will
 * be enabled. */
#define RAW_REGISTER
#include "hardware.h"


/* This register records the maximum ADC reading since it was last read. */
#define REGISTER_MAX_ADC_RAW    0x1400C000



/* Routine for printing an error message complete with associated file name
 * and line number. */

void print_error(const char * Message, const char * FileName, int LineNumber)
{
    /* Large enough not to really worry about overflow.  If we do generate a
     * silly message that's too big, then that's just too bad. */
    const int MESSAGE_LENGTH = 512;
    int Error = errno;
    char ErrorMessage[MESSAGE_LENGTH];
    
    int Count = snprintf(ErrorMessage, MESSAGE_LENGTH,
        "%s (%s, %d)", Message, FileName, LineNumber);
    if (errno != 0)
    {
        /* This is very annoying: strerror() is not not necessarily thread
         * safe ... but not for any compelling reason, see:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00101.html
         * and the rather unhelpful reply:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00108.html
         *
         * On the other hand, the recommended routine strerror_r() is
         * inconsistently defined -- depending on the precise library and its
         * configuration, it returns either an int or a char*.  Oh dear.
         *
         * Ah well.  We go with the GNU definition, so here is a buffer to
         * maybe use for the message. */
        char StrError[256];
        snprintf(ErrorMessage + Count, MESSAGE_LENGTH - Count,
            ": (%d) %s", Error, strerror_r(Error, StrError, sizeof(StrError)));
    }
    printf("%s\n", ErrorMessage);
}


/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/


/* Device handles. */
static int DevCfg = -1;     /* /dev/libera.cfg  General configuration. */
static int DevAdc = -1;     /* /dev/libera.adc  ADC configuration. */
static int DevDsc = -1;     /* /dev/libera.dsc  Signal conditioning i/f. */
static int DevEvent = -1;   /* /dev/libera.event    Event signalling. */
static int DevPm = -1;      /* /dev/libera.pm   Postmortem data. */
static int DevSa = -1;      /* /dev/libera.sa   Slow acquisition. */
static int DevDd = -1;      /* /dev/libera.dd   Turn by turn data. */
#ifdef RAW_REGISTER
static int DevMem = -1;     /* /dev/mem         Direct register access. */
#endif


/* Whether the Libera Brilliance option is installed.  This enables
 * completely different handling of attenuators and 16 bit ADC. */
static bool LiberaBrilliance = false;

/* The ADC nominally returns 16 bits (signed short) through the interface
 * provided here, but there are (at least) two types of ADC available: one
 * provides 12 bits, the other 16.  This value records how many bits need to
 * be corrected. */
static int AdcExcessBits = 4;



/*****************************************************************************/
/*                                                                           */
/*                      Miscellaneous Support Routines.                      */
/*                                                                           */
/*****************************************************************************/


/* Wrappers for reading and writing device driver configuration values.  Each
 * value is an integer identified by an id named in the form LIBERA_CFG_... */

static bool ReadCfgValue(int Index, int &Result)
{
    libera_cfg_request_t Request;
    Request.idx = Index;
    bool Ok = TEST_(ioctl, DevCfg, LIBERA_IOC_GET_CFG, &Request);
    if (Ok)
        Result = Request.val;
    return Ok;
}

static bool WriteCfgValue(int Index, int Value)
{
    libera_cfg_request_t Request;
    Request.idx = Index;
    Request.val = Value;
    return TEST_(ioctl, DevCfg, LIBERA_IOC_SET_CFG, &Request);
}


bool WriteInterlockParameters(
    LIBERA_ILKMODE mode,
    int Xlow, int Xhigh, int Ylow, int Yhigh,
    int overflow_limit, int overflow_dur,
    int gain_limit)
{
    /* Match the overflow limit setting to the actual number of bits provided
     * by the DSC.  Do this here allows the rest of the system to believe
     * everything is 16 bits. */
    overflow_limit >>= AdcExcessBits;
    return
        WriteCfgValue(LIBERA_CFG_ILK_MODE,           mode)  &&
        WriteCfgValue(LIBERA_CFG_ILK_XLOW,           Xlow)  &&
        WriteCfgValue(LIBERA_CFG_ILK_XHIGH,          Xhigh)  &&
        WriteCfgValue(LIBERA_CFG_ILK_YLOW,           Ylow)  &&
        WriteCfgValue(LIBERA_CFG_ILK_YHIGH,          Yhigh)  &&
        WriteCfgValue(LIBERA_CFG_ILK_OVERFLOW_LIMIT, overflow_limit)  &&
        WriteCfgValue(LIBERA_CFG_ILK_OVERFLOW_DUR,   overflow_dur)  &&
        /* It is important that this configuration value is written last, as
         * it turns out that nothing is written to hardware until this value
         * is written.  Eww: it would be better to have an explicit call if
         * that the way things should be. */
        WriteCfgValue(LIBERA_CFG_ILK_GAIN_LIMIT,     gain_limit);
}


bool WriteCalibrationSettings(
    int Kx, int Ky, int Kq, int Xoffset, int Yoffset)
{
    return
        WriteCfgValue(LIBERA_CFG_KX, Kx)  &&
        WriteCfgValue(LIBERA_CFG_KY, Ky)  &&
        WriteCfgValue(LIBERA_CFG_XOFFSET, Xoffset)  &&
        WriteCfgValue(LIBERA_CFG_YOFFSET, Yoffset);
}


bool SetMachineClockTime()
{
    libera_HRtimestamp_t Timestamp;
    Timestamp.mt = 0;
    Timestamp.phase = 0;
    return TEST_(ioctl, DevEvent, LIBERA_EVENT_SET_MT, &Timestamp);
}


bool SetSystemClockTime(const struct timespec & NewTime)
{
    libera_HRtimestamp_t Timestamp;
    Timestamp.st = NewTime;
    return TEST_(ioctl, DevEvent, LIBERA_EVENT_SET_ST, &Timestamp);
}


bool GetClockState(bool &LmtdLocked, bool &LstdLocked)
{
    int LmtdLockedInt = 0;
    int LstdLockedInt = 0;
    bool Ok =
        ReadCfgValue(LIBERA_CFG_MCPLL, LmtdLockedInt)  &&
        ReadCfgValue(LIBERA_CFG_SCPLL, LstdLockedInt);
    /* If either call fails the corresponding value will be left as false:
     * this is a sensible default value to return on failure. */
    LmtdLocked = LmtdLockedInt;
    LstdLocked = LstdLockedInt;
    return Ok;
}




/*****************************************************************************/
/*                                                                           */
/*                   Reading waveform data from the FPGA.                    */
/*                                                                           */
/*****************************************************************************/

/* Calling lseek() on /dev/libera.dd behaves quite differently depending on
 * how the whence parameter is set.  The following definitions document the
 * available options. */
#define LIBERA_SEEK_ST  SEEK_SET    // System clock
#define LIBERA_SEEK_MT  SEEK_CUR    // Machine clock
#define LIBERA_SEEK_TR  SEEK_END    // Trigger point (offset is ignored)


size_t ReadWaveform(
    int Decimation, size_t WaveformLength, LIBERA_ROW * Data,
    LIBERA_TIMESTAMP & Timestamp)
{
    const int ReadSize = sizeof(LIBERA_ROW) * WaveformLength;
    int Read;
    bool Ok =
        TEST_(ioctl, DevDd, LIBERA_IOC_SET_DEC, &Decimation)  &&
        TEST_(lseek, DevDd, 0, LIBERA_SEEK_TR)  &&
        TEST_IO(Read, read, DevDd, Data, ReadSize) &&
        TEST_(ioctl, DevDd, LIBERA_IOC_GET_DD_TSTAMP, &Timestamp);

    return Ok ? Read / sizeof(LIBERA_ROW) : 0;
}


size_t ReadPostmortem(
    size_t WaveformLength, LIBERA_ROW * Data, LIBERA_TIMESTAMP & Timestamp)
{
    const int ReadSize = sizeof(LIBERA_ROW) * WaveformLength;
    int Read;
    bool Ok =
        /* Very odd design in the driver: the postmortem waveform isn't
         * actually read until we do this ioctl!  This really isn't terribly
         * sensible, but never mind, that's how it works at the moment... */
        TEST_(ioctl, DevEvent, LIBERA_EVENT_ACQ_PM)  &&
        TEST_IO(Read, read, DevPm, Data, ReadSize)  &&
        TEST_(ioctl, DevPm, LIBERA_IOC_GET_PM_TSTAMP, &Timestamp);
    
    return Ok  &&  Read != -1  ?  Read / sizeof(LIBERA_ROW)  :  0;
}


bool ReadAdcWaveform(ADC_DATA &Data)
{
    size_t Read;
    bool Ok =
        TEST_IO(Read, read, DevAdc, Data, sizeof(ADC_DATA))  &&
        TEST_OK(Read == sizeof(ADC_DATA));
    if (Ok  &&  AdcExcessBits > 0)
    {
        /* Normalise all of the ADC data to 16 bits. */
        for (int i = 0; i < ADC_LENGTH; i ++)
        {
            for (int j = 0; j < 4; j ++)
                Data[i][j] <<= AdcExcessBits;
        }
    }
    return Ok;
}


bool ReadSlowAcquisition(ABCD_ROW &ButtonData, XYQS_ROW &PositionData)
{
    libera_atom_sa_t Result;
    int Read;
    bool Ok =
        TEST_IO(Read, read, DevSa, &Result, sizeof(libera_atom_sa_t))  &&
        TEST_OK(Read == sizeof(libera_atom_sa_t));
    if (Ok)
    {
        ButtonData.A = Result.Va;
        ButtonData.B = Result.Vb;
        ButtonData.C = Result.Vc;
        ButtonData.D = Result.Vd;
        PositionData.X = Result.X;
        PositionData.Y = Result.Y;
        PositionData.Q = Result.Q;
        PositionData.S = Result.Sum;
    }
    return Ok;
}


int ReadMaxAdc()
{
    unsigned int MaxAdc = 0;
    ReadRawRegister(REGISTER_MAX_ADC_RAW, MaxAdc);
    return MaxAdc << AdcExcessBits;
}



bool SetEventMask(int EventMask)
{
    return TEST_(ioctl, DevEvent, LIBERA_EVENT_SET_MASK, &EventMask);
}


/* Reads a single event from the event queue. */

bool ReadEvent(int &EventId, int &Parameter)
{
    libera_event_t Event;
    int Read = read(DevEvent, &Event, sizeof(Event));
    if (Read == sizeof(Event))
    {
        EventId = Event.id;
        Parameter = Event.param;
        return true;
    }
    else if (Read == 0)
        /* Odd.  Looks like every successful read is followed by a failed
         * read.  This appears to be a minor bug in the 1.46 device driver
         * (tests are done in the wrong order); easy to just ignore this. */
        return false;
    else
    {
        /* This really really isn't supposed to happen, you know: the
         * device takes care to return multiples of sizeof(Event)!  Well,
         * all we can do now is fill up the log file... */
        printf("Reading /dev/libera.event unexpectedly returned %d (%d)\n",
            Read, errno);
        return false;
    }
}



/*****************************************************************************/
/*                                                                           */
/*                            DSC Direct Access                              */
/*                                                                           */
/*****************************************************************************/


/* The following DSC offsets are all relative to the base of the Libera EBPP
 * FPGA address space.  All the DSC device offsets are relative to the ADC
 * block starting at this address when addressed through /dev/libera.dsc. */
#define DSC_DEVICE_OFFSET   0x8000

/* General control registers. */
#define DSC_DOUBLE_BUFFER       0xC024  // Double buffer control register
                                      
#define DSC_FILTER_DELAY        0xC028  // Analogue to digitial filter delay
#define DSC_HISTORY_MARKER      0xC030  // History marker origin and delay
#define DSC_SWITCH_DIVIDER      0xC038  // Switch division and trigger select
#define DSC_SWITCH_DELAY        0xC03C  // Switch delay control

/* Double buffered blocks.  For each block the name <name>_DB identifies the
 * length of the sub-block and the double-buffer division point. */
#define DSC_ATTENUATORS         0xC008  // Attenuator control registers
#define DSC_SWITCH_PATTERN      0xC800  // Switch sequencing pattern
#define DSC_PHASE_COMP          0xE800  // Phase compensation coefficients
#define DSC_SWITCH_DEMUX        0xF000  // Switch demultiplex coefficients

#define DSC_ATTENUATORS_DB      0x0008
#define DSC_SWITCH_PATTERN_DB   0x0400
#define DSC_PHASE_COMP_DB       0x0200
#define DSC_SWITCH_DEMUX_DB     0x0400


/* We somewhat arbitrarily constrain the switch pattern API to a maximum of
 * 16 switches.  There are many constraints on the sequence of switches,
 * which makes providing any serious amount of choice a futile exercise.
 *  1. There are only 16 possible switch positions.  Conceivably there might
 *     be arguments for repeating the same individual switch, but it seems
 *     implausiable.
 *  2. Switching produces strong harmonics which have to be filtered out by
 *     carefully chosen filters programmed into the FPGA: this reduces the
 *     usefulness of being able to change the switching sequence.
 *  3. Switching sequences need to be a power of 2 in length to fit into the
 *     (gratuitously enormous) switch memory -- this strongly constrains the
 *     possibilities for strange sequences. */


/* The entire double-buffered state is mirrored here and written when commit
 * is requested. */
static int Attenuation;

/* Records the currently selected switching pattern. */
static char SwitchPattern[MAX_SWITCH_SEQUENCE];

/* Records the current array of phase and amplitude and of demultiplexing (and
 * crosstalk) compensation values as raw processed values ready to be written
 * to hardware. */
static int RawSwitchDemux[DSC_SWITCH_DEMUX_DB/sizeof(int)];
static int RawPhaseComp[DSC_PHASE_COMP_DB/sizeof(int)];

/* Returns the offset appropriate to the selected block depending on the
 * state of the double buffer select flag.  When DoubleBufferSelect is zero
 * the bottom block is active and we must write to the top block, and vice
 * versa. */
#define DOUBLE_BUFFER(DoubleBufferSelect, block)    \
    (DoubleBufferSelect ? block : block + block##_DB)

/* For each of the four double buffered blocks we maintain a dirty counter.
 * This is set to 2 when the data is modified and decremented each time the
 * double buffer state is written to FPGA until the counter reaches zero:
 * this ensures that the updated state is written to *both* halves of the
 * double buffer. */
static int AttenuationDirty = 0;
static int SwitchPatternDirty = 0;
static int SwitchDemuxDirty = 0;
static int PhaseCompDirty = 0;

/* This macro marks the stage as dirty. */
#define MARK_DIRTY(action) \
    action##Dirty = 2
/* This macro performs the dirty check. */
#define CHECK_DIRTY(action) \
    if (action##Dirty > 0) \
        action##Dirty -= 1; \
    else \
        return true


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      Internal DSC support routines.                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Discovers whether this is a Libera Brillance system. */

static bool DiscoverBrilliance()
{
    /* Try to read the iTech FPGA feature configuration register to
     * discover whether this is a Brilliance Libera.  We do the ioctl by hand
     * here, rather than calling ReadCfgValue(), to avoid generating an error
     * message if the the requested feature isn't available. */
    libera_cfg_request_t req;
    req.idx = LIBERA_CFG_FEATURE_ITECH;
    if (ioctl(DevCfg, LIBERA_IOC_GET_CFG, &req) == -1)
        /* The feature register isn't present, so this can't possibly be
         * a Brilliance Libera. */
        LiberaBrilliance = false;
    else
        LiberaBrilliance = LIBERA_IS_BRILLIANCE(req.val);

    if (LiberaBrilliance)
        printf("Libera Brilliance detected\n");
    
    /* If the LiberaBrilliance flag is set then the ADC is 16 bits, otherwise
     * we're operating an older Libera with 12 bits.  We actually record and
     * use the excess bits which need to be handled specially. */
    AdcExcessBits = 16 - (LiberaBrilliance ? 16 : 12);
    return true;
}


static bool ReadDscWords(int offset, void *words, int length)
{
    /* Correct for DSC device base address. */
    offset -= DSC_DEVICE_OFFSET;
    int Read;
    return 
        TEST_(lseek, DevDsc, offset, SEEK_SET)  &&
        TEST_IO(Read, read, DevDsc, words, length)  &&
        TEST_OK(Read == length);
}


/* Writes a block of words to the DSC.  Again is assumed to always work (but
 * is noisy if things go wrong), and offsets are relative to the DSC area. */

static bool WriteDscWords(int offset, void *words, int length)
{
    /* Correct for DSC device base address. */
    offset -= DSC_DEVICE_OFFSET;
    int Written;
    return
        TEST_(lseek, DevDsc, offset, SEEK_SET)  &&
        TEST_IO(Written, write, DevDsc, words, length) &&
        TEST_OK(Written == length);
}



static bool ReadDscWord(int offset, int &word)
{
    return ReadDscWords(offset, &word, sizeof(int));
}


static bool WriteDscWord(int offset, int word)
{
    return WriteDscWords(offset, &word, sizeof(word));
}



/* Writes the attenuator state to the currently selected buffer. */

static bool WriteAttenuatorState(int Offset)
{
    CHECK_DIRTY(Attenuation);
        
    int AttenuatorWords[2];
    if (LiberaBrilliance)
    {
        /* For libera brilliance there are only four attenuators to set, each
         * using six bits.  We don't use the bottom bit (as 0.5dB steps
         * aren't that useful), and for some reason the bits are
         * complemented. */
        memset(AttenuatorWords, ~(Attenuation << 1), 4);
        AttenuatorWords[1] = 0;
    }
    else
    {
        /* For normal libera we split the attenuator value evenly across two
         * attenuators per channel. */
        char Atten1 = Attenuation / 2;
        char Atten2 = Attenuation - Atten1;
        char OneWord[4] = { Atten2, Atten1, Atten2, Atten1 };
        AttenuatorWords[1] = AttenuatorWords[0] = *(int*)OneWord;
    }
    return WriteDscWords(Offset, AttenuatorWords, sizeof(AttenuatorWords));
}


/* The switch history mark is written into bits 19:16 of the history marker
 * control register and a programmable delay from switch change to the marker
 * is written into bits 15:0.  For the moment we hard code zero into the
 * history delay. */

static bool WriteHistoryMark()
{
    return
        WriteDscWord(DSC_HISTORY_MARKER, (SwitchPattern[0] & 0xF) << 16);
}


/* The sequence of switches is repeated to fill the complete switch pattern
 * block. */

static bool WriteSwitchesState(int Offset)
{
    CHECK_DIRTY(SwitchPattern);
    
    /* Two switches per byte.  Pack the switches. */
    char Template[MAX_SWITCH_SEQUENCE/2];
    for (unsigned int i = 0; i < sizeof(Template); i++)
        Template[i] = SwitchPattern[2*i] | (SwitchPattern[2*i + 1] << 4);

    /* Now prepare the full block before writing it to the DSC device. */
    char SwitchPatternBlock[DSC_SWITCH_PATTERN_DB];
    for (int i = 0; i < DSC_SWITCH_PATTERN_DB; i += sizeof(Template))
        memcpy(SwitchPatternBlock + i, Template, sizeof(Template));

    return
        /* Write out the new DSC switch control block ready to be activated
         * when the double buffer is swapped. */
        WriteDscWords(Offset, SwitchPatternBlock, DSC_SWITCH_PATTERN_DB)  &&
        /* Finally ensure that history mark is updated.  This last step is,
         * alas, out of sync with everything else as it is not double
         * buffered.  Too bad: it won't have much effect. */
        WriteHistoryMark();
}


static bool WritePhaseState(int Offset)
{
    CHECK_DIRTY(PhaseComp);
    return WriteDscWords(Offset, RawPhaseComp, DSC_PHASE_COMP_DB);
}


static bool WriteDemuxState(int Offset)
{
    CHECK_DIRTY(SwitchDemux);
    return WriteDscWords(Offset, RawSwitchDemux, DSC_SWITCH_DEMUX_DB);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                     Published DSC Interface Routines.                     */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


bool Brilliance()
{
    return LiberaBrilliance;
}


bool WriteAttenuation(int NewAttenuation)
{
    if (0 <= NewAttenuation  &&  NewAttenuation <= 62)
    {
        Attenuation = NewAttenuation;
        MARK_DIRTY(Attenuation);
        return true;
    }
    else
    {
        printf("Invalid attenuator value %d\n", NewAttenuation);
        return false;
    }
}


bool WriteSwitchSequence(
    const SWITCH_SEQUENCE NewSwitches, unsigned int NewLength)
{
    /* The pattern length must first be a power of 2 and secondly be within
     * range. */
    if (NewLength > MAX_SWITCH_SEQUENCE)
        printf("Switch pattern length %u too long\n", NewLength);
    else if ((NewLength & -NewLength) != NewLength  ||  NewLength == 0)
        /* Note: the trick (x&-x) isolates the most sigificant bit of x. */
        printf("Switch pattern length %u must be power of 2\n", NewLength);
    else
    {
        /* Copy over the new switch pattern, repeating as necessary to fill
         * up to the standard length.  Only the bottom four bits of each
         * switch are used. */
        for (unsigned int i = 0; i < MAX_SWITCH_SEQUENCE; i += NewLength)
            for (unsigned int j = 0; j < NewLength; j ++)
                SwitchPattern[i+j] = NewSwitches[j] & 0xF;
        MARK_DIRTY(SwitchPattern);
        return true;
    }
    return false;
}


/* The mapping from PHASE_ARRAY values to FPGA addresses is reasonably
 * straightforward: given
 *      n = switch value
 *      i = channel index
 *      k = filter index
 * then the target address (as an index into an integer array) for
 * Array[i][k] has the following pattern:
 *
 *  bit:    6    5   4      1     0
 *      --+--------+----------+------+
 *        | i[1:0] |  n[3:0]  | k[0] |
 *      --+--------+----------+------+ */

void WritePhaseArray(int Switch, const PHASE_ARRAY Array)
{
    for (int i = 0; i < BUTTON_COUNT; i ++)
    {
        int Base = ((Switch & 0xF) << 1) | (i << 5);
        RawPhaseComp[Base]   = Array[i][0];
        RawPhaseComp[Base+1] = Array[i][1];
    }
    MARK_DIRTY(PhaseComp);
}


/* The mapping from DEMUX_ARRAY values to FPGA addresses is slightly
 * uncomfortable: given
 *      n = switch value
 *      i = input channel index
 *      j = output button index
 * then the target address (as an index into an integer array) for
 * Array[j][i] has the following pattern: 
 *
 *  bit:    7    6     5    4  ..  1     0
 *      --+--------+------+----------+------+
 *        | j[1:0] | i[1] |  n[3:0]  | i[0] |
 *      --+--------+------+----------+------+ */
 
void WriteDemuxArray(int Switch, const DEMUX_ARRAY Array)
{
    for (int j = 0; j < BUTTON_COUNT; j ++)
    {
        int Base = ((Switch & 0xF) << 1) | (j << 6);
        for (int i = 0; i < BUTTON_COUNT; i ++)
            RawSwitchDemux[Base | (i & 1) | ((i & 2) << 4)] = Array[j][i];
    }
    MARK_DIRTY(SwitchDemux);
}



/* Commits all written double-buffer state by switching double buffers. */

bool CommitDscState()
{
    int Buffer;
    return
        /* Pick up which double buffer is currently active. */
        ReadDscWord(DSC_DOUBLE_BUFFER, Buffer)  &&

        /* Write our current (new) state into the current writeable buffer. */
        WriteAttenuatorState(DOUBLE_BUFFER(Buffer, DSC_ATTENUATORS))  &&
        WriteSwitchesState  (DOUBLE_BUFFER(Buffer, DSC_SWITCH_PATTERN))  &&
        WritePhaseState     (DOUBLE_BUFFER(Buffer, DSC_PHASE_COMP))  &&
        WriteDemuxState     (DOUBLE_BUFFER(Buffer, DSC_SWITCH_DEMUX))  &&

        /* Swap the new buffer into place: in effect, an atomic write. */
        WriteDscWord(DSC_DOUBLE_BUFFER, Buffer^1);
}


/* The switch trigger source is controlled by the top bit of the
 * turn-by-turn divider register. */

bool WriteSwitchTriggerSelect(bool ExternalTrigger)
{
    int DividerValue;
    return
        ReadDscWord(DSC_SWITCH_DIVIDER, DividerValue)  &&
        WriteDscWord(DSC_SWITCH_DIVIDER,
            (DividerValue & 0x7FFFFFFF) | (ExternalTrigger << 31));
}


/* The delay on the switch clock source is programmed into the bottom ten
 * bits of the delay control register. */

bool WriteSwitchTriggerDelay(int Delay)
{
    int DelayControl;
    return
        ReadDscWord(DSC_SWITCH_DELAY, DelayControl)  &&
        WriteDscWord(DSC_SWITCH_DELAY,
            (DelayControl & 0xFFFF0000) | (Delay & 0x3FF));
}



#ifdef RAW_REGISTER
/*****************************************************************************/
/*                                                                           */
/*                           Raw Register Access                             */
/*                                                                           */
/*****************************************************************************/

/* Uses /dev/mem to directly access a specified hardware address. */


/* Paging information. */
static unsigned int OsPageSize;         // 0x1000
static unsigned int OsPageMask;         // 0x0FFF


static unsigned int * MapRawRegister(unsigned int Address)
{
    char * MemMap = (char *) mmap(
        0, OsPageSize, PROT_READ | PROT_WRITE, MAP_SHARED,
        DevMem, Address & ~OsPageMask);
    if (MemMap == MAP_FAILED)
    {
        perror("Unable to map register into memory");
        return NULL;
    }
    else
        return (unsigned int *)(MemMap + (Address & OsPageMask));
}

static void UnmapRawRegister(unsigned int *MappedAddress)
{
    void * BaseAddress = (void *) (
        ((unsigned int) MappedAddress) & ~OsPageMask);
    munmap(BaseAddress, OsPageSize);
}


bool WriteRawRegister(unsigned int Address, unsigned int Value)
{
    unsigned int * Register = MapRawRegister(Address);
    if (Register == NULL)
        return false;
    else
    {
        *Register = Value;
        UnmapRawRegister(Register);
        return true;
    }
}


bool ReadRawRegister(unsigned int Address, unsigned int &Value)
{
    unsigned int * Register = MapRawRegister(Address);
    if (Register == NULL)
        return false;
    else
    {
        Value = *Register;
        UnmapRawRegister(Register);
        return true;
    }
}


#endif


/*****************************************************************************/
/*                                                                           */
/*                      Initialisation and Shutdown                          */
/*                                                                           */
/*****************************************************************************/



bool InitialiseHardware()
{
#ifdef RAW_REGISTER
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;
#endif
    
    return
        /* Just open all the Libera devices and check for the presence of the
         * Brilliance option. */
        TEST_IO(DevCfg,   open, "/dev/libera.cfg",   O_RDWR)  &&
        TEST_IO(DevAdc,   open, "/dev/libera.adc",   O_RDONLY)  &&
        TEST_IO(DevDsc,   open, "/dev/libera.dsc",   O_RDWR | O_SYNC)  &&
        TEST_IO(DevEvent, open, "/dev/libera.event", O_RDWR)  &&
        TEST_IO(DevPm,    open, "/dev/libera.pm",    O_RDONLY)  &&
        TEST_IO(DevSa,    open, "/dev/libera.sa",    O_RDONLY)  &&
        TEST_IO(DevDd,    open, "/dev/libera.dd",    O_RDONLY)  &&
#ifdef RAW_REGISTER
        TEST_IO(DevMem,   open, "/dev/mem", O_RDWR | O_SYNC)  &&
#endif
        DiscoverBrilliance();
}
