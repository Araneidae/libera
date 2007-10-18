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


/* Libera device interface reimplemented through CSPI. */

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


#define CSPI_(command, args...) \
    ( { int error = (command)(args); \
        if (error != CSPI_OK) \
            printf("CSPI error in %s (%s, %d): %s\n", \
                #command, __FILE__, __LINE__, cspi_strerror(error)); \
        error == CSPI_OK; } )

#if 0
/* This version of the CSPI macro logs every CSPI call! */
#undef CSPI_
#define CSPI_(command, args...) \
    ( { printf("%s", "CSPI " #command " (" #args ") => "); \
        int error = (command)(args); \
        printf("%d\n", error); \
        if (error != CSPI_OK) \
            printf("CSPI error in %s: %s\n", #command, cspi_strerror(error)); \
        error == CSPI_OK; } )
#endif


/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/

/* The following handles manage our connection to CSPI. */

/* This is the main environment handle needed for establishing the initial
 * connection and to manage the other active connections. */
static CSPIHENV CspiEnv = NULL;
/* Connection to ADC rate buffer. */
static CSPIHCON CspiConAdc = NULL;
/* Connection to turn-by-turn or decimated data buffer, also known as the
 * "data on demand" (DD) data source. */
static CSPIHCON CspiConDd = NULL;
/* Connection to slow acquisition data source, updating at just over 10Hz. */
static CSPIHCON CspiConSa = NULL;
/* Connection to 16384 point postmortem buffer. */
static CSPIHCON CspiConPm = NULL;
/* Connection handle used to configure CSPI event deliver. */
static CSPIHCON EventSource = NULL;


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



bool WriteInterlockParameters(
    CSPI_ILKMODE mode,
    int Xlow, int Xhigh, int Ylow, int Yhigh,
    int overflow_limit, int overflow_dur,
    int gain_limit)
{
    /* Match the overflow limit setting to the actual number of bits provided
     * by the DSC.  Do this here allows the rest of the system to believe
     * everything is 16 bits. */
    overflow_limit >>= AdcExcessBits;
    
    CSPI_ENVPARAMS EnvParams;
    EnvParams.ilk.mode = mode;
    EnvParams.ilk.Xlow = Xlow;
    EnvParams.ilk.Xhigh = Xhigh;
    EnvParams.ilk.Ylow = Ylow;
    EnvParams.ilk.Yhigh = Yhigh;
    EnvParams.ilk.overflow_limit = overflow_limit;
    EnvParams.ilk.overflow_dur = overflow_dur;
    EnvParams.ilk.gain_limit = gain_limit;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_ILK);
}


bool WriteCalibrationSettings(
    int Kx, int Ky, int Kq, int Xoffset, int Yoffset)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.Kx = Kx;
    EnvParams.Ky = Ky;
    EnvParams.Xoffset = Xoffset;
    EnvParams.Yoffset = Yoffset;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams,
        CSPI_ENV_KX | CSPI_ENV_KY | CSPI_ENV_XOFFSET | CSPI_ENV_YOFFSET);
}


bool SetMachineClockTime()
{
    CSPI_SETTIMESTAMP Timestamp;
    Timestamp.mt = 0;
    Timestamp.phase = 0;
    return CSPI_(cspi_settime, CspiEnv, &Timestamp, CSPI_TIME_MT);
}


bool SetSystemClockTime(struct timespec & NewTime)
{
    CSPI_SETTIMESTAMP Timestamp;
    Timestamp.st = NewTime;
    return CSPI_(cspi_settime, CspiEnv, &Timestamp, CSPI_TIME_ST);
}


bool GetClockState(bool &LmtdLocked, bool &LstdLocked)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_PLL);
    if (Ok)
    {
        LmtdLocked = EnvParams.pll.mc;
        LstdLocked = EnvParams.pll.sc;
    }
    return Ok;
}




/*****************************************************************************/
/*                                                                           */
/*                   Reading waveform data from the FPGA.                    */
/*                                                                           */
/*****************************************************************************/


#define READ_CHUNK_SIZE         65536
#define CHUNK_SIZE(Length) \
    ((Length) > READ_CHUNK_SIZE ? READ_CHUNK_SIZE : (Length))

size_t ReadWaveform(
    int Decimation, size_t WaveformLength, LIBERA_ROW * Data,
    CSPI_TIMESTAMP & Timestamp)
{
    CSPI_CONPARAMS_DD ConParams;
    ConParams.dec = Decimation;
    unsigned long long Offset = 0;

    /* Don't read more than a single chunk at a time: managing the blocks
     * like this prevents us from starving other activities. */
    size_t ChunkSize = CHUNK_SIZE(WaveformLength);
    
    size_t TotalRead;
    bool Ok =
        // Set the decimation mode
        CSPI_(cspi_setconparam, CspiConDd,
            (CSPI_CONPARAMS *)&ConParams, CSPI_CON_DEC)  &&
        // Seek to the trigger point
        CSPI_(cspi_seek, CspiConDd, &Offset, CSPI_SEEK_TR)  &&
        // Read the data
        CSPI_(cspi_read_ex, CspiConDd, Data, ChunkSize, &TotalRead, NULL)  &&
        // Finally read the timestamp: needs to be done after data read.
        CSPI_(cspi_gettimestamp, CspiConDd, &Timestamp);

    /* Check if we need to do multiple reads (and if we managed to perform a
     * complete read in the first place). */
    if (Ok  &&  TotalRead == ChunkSize  &&  ChunkSize < WaveformLength)
    {
        /* One chunk wasn't enough.  Unfortunately there is a quirk in the
         * driver: repeated reads after CSPI_SEEK_TR don't actually give us
         * successive data blocks!  Instead we'll need to perform an absolute
         * seek: then we can read the rest in sequence.
         *    In this extra bit of code we are rather less fussy about
         * errors: we've already got good data in hand, so there's no point
         * in not returning what we have if anything subsequent fails. */
        Offset = Timestamp.mt + ChunkSize;
        if (CSPI_(cspi_seek, CspiConDd, &Offset, CSPI_SEEK_MT))
        {
            int cspi_rc;
            do
            {
                /* Count off the chunk just read and prepare for the next. */
                WaveformLength -= ChunkSize;
                Data += ChunkSize;
                ChunkSize = CHUNK_SIZE(WaveformLength);
                /* Read incoming chunks until either we've read everything or
                 * a read comes up short. */
                size_t Read;
                cspi_rc = cspi_read_ex(
                    CspiConDd, Data, ChunkSize, &Read, NULL);
                if (cspi_rc == CSPI_OK  ||  cspi_rc == CSPI_W_INCOMPLETE)
                    TotalRead += Read;
            } while (cspi_rc == CSPI_OK  &&  ChunkSize < WaveformLength);
        }
    }

    return Ok ? TotalRead : 0;
}


size_t ReadPostmortem(
    size_t WaveformLength, LIBERA_ROW * Data, CSPI_TIMESTAMP & Timestamp)
{
    size_t Read;
    bool Ok =
        CSPI_(cspi_read_ex, CspiConPm, Data, 16384, &Read, NULL)  &&
        // Finally read the timestamp: needs to be done after data read.
        CSPI_(cspi_gettimestamp, CspiConDd, &Timestamp);

    return Ok ? Read : 0;
}


bool ReadAdcWaveform(ADC_DATA &Data)
{
    size_t Read;
    bool Ok =
        CSPI_(cspi_read_ex, CspiConAdc, &Data, ADC_LENGTH, &Read, NULL)  &&
        Read == 1024;
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
    CSPI_SA_ATOM Result;
    if (CSPI_(cspi_get, CspiConSa, &Result))
    {
        ButtonData.A = Result.Va;
        ButtonData.B = Result.Vb;
        ButtonData.C = Result.Vc;
        ButtonData.D = Result.Vd;
        PositionData.X = Result.X;
        PositionData.Y = Result.Y;
        PositionData.Q = Result.Q;
        PositionData.S = Result.Sum;
        return true;
    }
    else
        return false;
}


int ReadMaxAdc()
{
    unsigned int MaxAdc = 0;
    ReadRawRegister(REGISTER_MAX_ADC_RAW, MaxAdc);
    return MaxAdc << AdcExcessBits;
}



bool ConfigureEventCallback(
    int EventMask, int (*Handler)(CSPI_EVENT*), void *Context)
{
    CSPI_CONPARAMS ConParams;
    ConParams.event_mask = EventMask;
    ConParams.handler = Handler;
    ConParams.user_data = Context;
    return CSPI_(cspi_setconparam, EventSource, &ConParams,
        CSPI_CON_EVENTMASK | CSPI_CON_HANDLER | CSPI_CON_USERDATA);
}



/*****************************************************************************/
/*                                                                           */
/*                            DSC Direct Access                              */
/*                                                                           */
/*****************************************************************************/


/* Handle to /dev/libera.dsc device, used for DSC interface. */
static int DevDsc = -1;

/* Whether the Libera Brilliance option is installed.  This enables
 * completely different handling of attenuators and 16 bit ADC. */
static bool LiberaBrilliance = false;


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


static bool InitialiseDSC()
{
    bool Ok =
        /* Open /dev/libera.dsc for signal conditioning control. */
        TEST_IO(DevDsc, "Unable to open /dev/libera.dsc",
            open, "/dev/libera.dsc", O_RDWR | O_SYNC)  &&
        /* Interrogate whether Libera Brilliance is installed. */
        TEST_(ioctl, DevDsc, LIBERA_DSC_GET_ADC, &LiberaBrilliance);
    /* If the LiberaBrilliance flag is set then the ADC is 16 bits, otherwise
     * we're operating an older Libera with 12 bits.  We actually record and
     * use the excess bits which need to be handled specially. */
    AdcExcessBits = 16 - (LiberaBrilliance ? 16 : 12);
    return Ok;
}


static bool ReadDscWords(int offset, void *words, int length)
{
    /* Correct for DSC device base address. */
    offset -= DSC_DEVICE_OFFSET;
    int Read;
    return 
        TEST_IO(Read, "Error seeking /dev/libera.dsc",
            lseek, DevDsc, offset, SEEK_SET)  &&
        TEST_IO(Read, "Error reading from /dev/libera.dsc",
            read, DevDsc, words, length)  &&
        TEST_OK(Read == length,
            "Read from DSC was incomplete (read %d of %d)\n", Read, length);
}


/* Writes a block of words to the DSC.  Again is assumed to always work (but
 * is noisy if things go wrong), and offsets are relative to the DSC area. */

static bool WriteDscWords(int offset, void *words, int length)
{
    /* Correct for DSC device base address. */
    offset -= DSC_DEVICE_OFFSET;
    int Written;
    return
        TEST_IO(Written, "Error seeking /dev/libera.dsc",
            lseek, DevDsc, offset, SEEK_SET)  &&
        TEST_IO(Written, "Error writing to /dev/libera.dsc",
            write, DevDsc, words, length) &&
        TEST_OK(Written == length,
            "Write to DSC was incomplete (wrote %d of %d)\n",
            Written, length);
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


/* The following handle to /dev/mem is held open for direct hardware register
 * access. */
static int DevMem = -1;
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



static bool InitialiseConnection(CSPIHCON &Connection, int Mode)
{
    CSPI_CONPARAMS ConParams;
    ConParams.mode = Mode;
    return 
        CSPI_(cspi_allochandle, CSPI_HANDLE_CON, CspiEnv, &Connection)  &&
        CSPI_(cspi_setconparam, Connection, &ConParams, CSPI_CON_MODE)  &&
        CSPI_(cspi_connect, Connection);
}


bool InitialiseHardware()
{
#ifdef RAW_REGISTER
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;
#endif
    
    CSPI_LIBPARAMS LibParams;
    LibParams.superuser = 1;    // Allow us to change settings!
    return
        /* First ensure that the library allows us to change settings! */
        CSPI_(cspi_setlibparam, &LibParams, CSPI_LIB_SUPERUSER)  &&
        /* Open the CSPI environment and then open CSPI handles for each of
         * the data channels we need. */
        CSPI_(cspi_allochandle, CSPI_HANDLE_ENV, 0, &CspiEnv)  &&
        CSPI_(cspi_allochandle, CSPI_HANDLE_CON, CspiEnv, &EventSource)  &&
        InitialiseConnection(CspiConAdc, CSPI_MODE_ADC)  &&
        InitialiseConnection(CspiConDd, CSPI_MODE_DD)  &&
        InitialiseConnection(CspiConSa, CSPI_MODE_SA)  &&
        InitialiseConnection(CspiConPm, CSPI_MODE_PM)  &&
        InitialiseDSC()  &&
#ifdef RAW_REGISTER
        /* Open /dev/mem for register access. */
        TEST_IO(DevMem, "Unable to open /dev/mem",
            open, "/dev/mem", O_RDWR | O_SYNC)  &&
#endif
        true;
}
