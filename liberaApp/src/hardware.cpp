/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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
#include <stdint.h>
#include <pthread.h>

#include "versions.h"

/* If RAW_REGISTER is defined then raw register access through /dev/mem will
 * be enabled. */
#include "hardware.h"


/* Feature registers used to identify special functionality. */
#define REGISTER_BUILD_NUMBER   0x14000008
#define REGISTER_DLS_FEATURE    0x14000018
#define REGISTER_ITECH_FEATURE  0x1400001C

/* This register has two functions:
 *  bits 29:16: program a delay on the external trigger.
 *  bits 15:14: PM trigger source selection:
 *      0 => External hardware PM trigger
 *      1 => Internal interlock check
 *      2,3 => Separate check on FA data programmed by extra registers. */
#define REGISTER_TRIG_DELAY     0x14004038
/* These two registers record the maximum ADC reading since they were last
 * read.  Unfortunately the DLS and iTech FPGAs use different registers. */
#define REGISTER_MAX_ADC_ITECH  0x14008004
#define REGISTER_MAX_ADC_DLS    0x1400C000
/* This register is used to set the turn by turn ADC overflow threshold. */
#define REGISTER_ADC_OVERFLOW   0x1400C004
/* Postmortem trigger ADC overflow registers. */
#define REGISTER_PM_ADC_LIMIT   0x1400C040  // ADC overflow threshold for PM
#define REGISTER_PM_ADC_TIME    0x1400C044  // ADC overflow duration for PM

/* These registers are used to access the triggered sum average. */
#define FA_OFFSET               0x1401C000  // Base of FA register area

#define REGISTER_FA_NSUMS       (FA_OFFSET + 0x024) // Number of samples
#define REGISTER_FA_SUM_LSW     (FA_OFFSET + 0x028) // Low 32 bits
#define REGISTER_FA_SUM_MSW     (FA_OFFSET + 0x02C) // High 32 bits of sum

/* Spike removal control registers. */
#define REGISTER_SR_ENABLE      (FA_OFFSET + 0x030) // 1 => removal enabled
#define REGISTER_SR_AVE_STOP    (FA_OFFSET + 0x034) // End point for average
#define REGISTER_SR_AVE_WIN     (FA_OFFSET + 0x038) // Length of average window
#define REGISTER_SR_SPIKE_START (FA_OFFSET + 0x03C) // Start of spike
#define REGISTER_SR_SPIKE_WIN   (FA_OFFSET + 0x040) // Length of spike window
#define REGISTER_SR_DEBUG       (FA_OFFSET + 0x044) // Debug register
#define REGISTER_SR_BUFFER      (FA_OFFSET + 0x800) // Debug buffer

/* Postmortem trigger position limit registers. */
#define REGISTER_PM_MINX        0x14024020  // PM trigger min X limit
#define REGISTER_PM_MAXX        0x14024024  // PM trigger max X limit
#define REGISTER_PM_MINY        0x14024028  // PM trigger min Y limit
#define REGISTER_PM_MAXY        0x1402402C  // PM trigger max Y limit


/* This bit is set in the ITECH register to enable DLS extensions. */
#define DLS_EXTENSION_BIT       (1 << 23)
/* The following bits in the DLS feature register define extensions. */
#define DLS_CORE_EXTENSIONS     (1 << 31)   // Must be set


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


/* This mutex is used to ensure serial access to hardware.  See also similar
 * code in interlock.cpp.
 *    Note that pthread_cleanup_{push,pop} need to be used here because we're
 * also using pthread_cancel, and if we're not meticulous about cleaning up
 * then orderly shutdown gets disrupted. */
static pthread_mutex_t hardware_mutex = PTHREAD_MUTEX_INITIALIZER;
static void Lock()         { TEST_0(pthread_mutex_lock(&hardware_mutex)); }
static void Unlock(void *) { TEST_0(pthread_mutex_unlock(&hardware_mutex)); }
#define LOCK()      Lock(); pthread_cleanup_push(Unlock, NULL)
#define UNLOCK()    pthread_cleanup_pop(true)
/*
#define LOCK()      \
    Lock(); pthread_cleanup_push(Unlock, NULL); printf("Locked %d\n",__LINE__)
#define UNLOCK()    \
    pthread_cleanup_pop(true); printf("Unlocked %d\n",__LINE__)
 */
#define LOCKED(result) \
    ( { \
        __typeof__(result) __result__; \
        LOCK(); \
        __result__ = (result); \
        UNLOCK(); \
        __result__; \
    } )



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


/* The ADC nominally returns 16 bits (signed short) through the interface
 * provided here, but there are (at least) two types of ADC available: one
 * provides 12 bits, the other 16.  This value records how many bits need to
 * be corrected. */
static int AdcExcessBits = 4;

/* Max ADC register read at SA rate.  We preallocate this to avoid
 * continually mapping the register! */
static unsigned int * RegisterMaxAdcRaw = NULL;

/* Number of turns per switch. */
static int TurnsPerSwitch;



/*****************************************************************************/
/*                                                                           */
/*                           Raw Register Access                             */
/*                                                                           */
/*****************************************************************************/

/* Uses /dev/mem to directly access a specified hardware address. */

#ifdef RAW_REGISTER

/* Paging information. */
static uint32_t OsPageSize;         // 0x1000
static uint32_t OsPageMask;         // 0x0FFF


static uint32_t * MapRawRegister(uint32_t Address)
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
        return (uint32_t *)(MemMap + (Address & OsPageMask));
}

static void UnmapRawRegister(uint32_t *MappedAddress)
{
    void * BaseAddress = (void *) (
        ((uint32_t) MappedAddress) & ~OsPageMask);
    munmap(BaseAddress, OsPageSize);
}

#else
static uint32_t * MapRawRegister(uint32_t Address)
{
    errno = 0;
    print_error("Cannot map registers into memory", __FILE__, __LINE__);
    return NULL;
}

static void UnmapRawRegister(uint32_t *MappedAddress) { }

#endif


static bool WriteRawRegister(
    uint32_t Address, uint32_t Value, uint32_t Mask = 0xFFFFFFFF)
{
    uint32_t * Register = MapRawRegister(Address);
    if (Register == NULL)
        return false;
    else
    {
        if (Mask != 0xFFFFFFFF)
            Value = (Value & Mask) | (*Register & ~Mask);
        *Register = Value;
        UnmapRawRegister(Register);
        return true;
    }
}


#if 0
static bool ReadRawRegister(uint32_t Address, uint32_t &Value)
{
    uint32_t * Register = MapRawRegister(Address);
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
/*                      Miscellaneous Support Routines.                      */
/*                                                                           */
/*****************************************************************************/


/* Wrappers for reading and writing device driver configuration values.  Each
 * value is an integer identified by an id named in the form LIBERA_CFG_... */

static bool ReadCfgValue(int Index, int &Result)
{
    libera_cfg_request_t Request;
    Request.idx = Index;
    bool Ok = TEST_IO(ioctl(DevCfg, LIBERA_IOC_GET_CFG, &Request));
    if (Ok)
        Result = Request.val;
    return Ok;
}

static bool WriteCfgValue(int Index, int Value)
{
    libera_cfg_request_t Request;
    Request.idx = Index;
    Request.val = Value;
    return TEST_IO(ioctl(DevCfg, LIBERA_IOC_SET_CFG, &Request));
}


bool WriteInterlockParameters(
    LIBERA_ILKMODE mode,
    int Xlow, int Xhigh, int Ylow, int Yhigh,
    int overflow_limit, int overflow_dur,
    int gain_limit)
{
    /* Match the overflow limit setting to the actual number of bits provided
     * by the DSC.  Doing this here allows the rest of the system to believe
     * everything is 16 bits. */
    overflow_limit >>= AdcExcessBits;
    return LOCKED(
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
        WriteCfgValue(LIBERA_CFG_ILK_GAIN_LIMIT,     gain_limit)  &&
#ifdef RAW_REGISTER
        /* Finally, if the DLS ADC overflow register is in use, write to that
         * as well: in this case the overflow_limit above is ignored. */
        IF_(DlsFpgaFeatures,
            WriteRawRegister(REGISTER_ADC_OVERFLOW, overflow_limit))  &&
#endif
        true);
}


bool WriteCalibrationSettings(int Kx, int Ky, int Xoffset, int Yoffset)
{
    return LOCKED(
        WriteCfgValue(LIBERA_CFG_KX, Kx)  &&
        WriteCfgValue(LIBERA_CFG_KY, Ky)  &&
        WriteCfgValue(LIBERA_CFG_XOFFSET, Xoffset)  &&
        WriteCfgValue(LIBERA_CFG_YOFFSET, Yoffset));
}


bool SetMachineClockTime()
{
    libera_HRtimestamp_t Timestamp;
    Timestamp.mt = 0;
    Timestamp.phase = 0;
    return TEST_IO(ioctl(DevEvent, LIBERA_EVENT_SET_MT, &Timestamp));
}


bool SetSystemClockTime(const struct timespec & NewTime)
{
    libera_HRtimestamp_t Timestamp;
    Timestamp.st = NewTime;
    return TEST_IO(ioctl(DevEvent, LIBERA_EVENT_SET_ST, &Timestamp));
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


bool WriteExternalTriggerDelay(int Delay)
{
    if (0 <= Delay  &&  Delay < 1<<12)
        return LOCKED(WriteRawRegister(
            REGISTER_TRIG_DELAY, Delay << 16, 0x0FFF0000));
    else
        return false;
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
    LIBERA_TIMESTAMP & Timestamp, int Offset)
{
    const int ReadSize = sizeof(LIBERA_ROW) * WaveformLength;
    int Read = 0;
    bool Ok = LOCKED(
        TEST_IO(ioctl(DevDd, LIBERA_IOC_SET_DEC, &Decimation))  &&
        TEST_IO(lseek(DevDd, Offset, LIBERA_SEEK_TR))  &&
        TEST_IO(Read = read(DevDd, Data, ReadSize)) &&
        TEST_IO(ioctl(DevDd, LIBERA_IOC_GET_DD_TSTAMP, &Timestamp)));

    return Ok ? Read / sizeof(LIBERA_ROW) : 0;
}


size_t ReadPostmortem(
    size_t WaveformLength, LIBERA_ROW * Data, LIBERA_TIMESTAMP & Timestamp)
{
    const int ReadSize = sizeof(LIBERA_ROW) * WaveformLength;
    int Read = 0;
    bool Ok = LOCKED(
        /* Very odd design in the driver: the postmortem waveform isn't
         * actually read until we do this ioctl!  This really isn't terribly
         * sensible, but never mind, that's how it works at the moment... */
        TEST_IO(ioctl(DevEvent, LIBERA_EVENT_ACQ_PM))  &&
        TEST_IO(Read = read(DevPm, Data, ReadSize))  &&
        TEST_IO(ioctl(DevPm, LIBERA_IOC_GET_PM_TSTAMP, &Timestamp)));
    
    return Ok  &&  Read != -1  ?  Read / sizeof(LIBERA_ROW)  :  0;
}


bool ReadAdcWaveform(ADC_DATA &Data)
{
    size_t Read = 0;
    bool Ok = LOCKED(
        TEST_IO(Read = read(DevAdc, Data, sizeof(ADC_DATA)))  &&
        TEST_OK(Read == sizeof(ADC_DATA)));
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
    int Read = 0;
    bool Ok = 
        TEST_IO(Read = read(DevSa, &Result, sizeof(libera_atom_sa_t)))  &&
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
    if (RegisterMaxAdcRaw == NULL)
        return 0;
    else
        return *RegisterMaxAdcRaw << AdcExcessBits;
}



bool SetEventMask(int EventMask)
{
    return TEST_IO(ioctl(DevEvent, LIBERA_EVENT_SET_MASK, &EventMask));
}


/* Reads events from the event queue, returning the number read. */

int ReadEvents(libera_event_t Events[], int MaxEventCount)
{
    int Read = read(DevEvent, Events, sizeof(libera_event_t) * MaxEventCount);
    return TEST_IO(Read) ? Read / sizeof(libera_event_t) : 0;
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
#define DSC_INTERLOCK_IIR_K     0xC034  // Interlock IIR coefficient
#define DSC_SWITCH_DIVIDER      0xC038  // Switch division and trigger select
#define DSC_SWITCH_DELAY        0xC03C  // Switch delay control

/* Double buffered blocks.  For each block the name <name>_DB identifies the
 * length of the sub-block and the double-buffer division point. */
#define DSC_ATTENUATORS         0xC008  // Attenuator control registers
#define DSC_ATTENUATORS_DB      0x0008
#define DSC_SWITCH_PATTERN      0xC800  // Switch sequencing pattern
#define DSC_SWITCH_PATTERN_DB   0x0400
#define DSC_PHASE_COMP          0xE800  // Phase compensation coefficients
#define DSC_PHASE_COMP_DB       0x0200
#define DSC_SWITCH_DEMUX        0xF000  // Switch demultiplex coefficients
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
static uint8_t SwitchPattern[MAX_SWITCH_SEQUENCE];

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


static bool ReadDscWords(int offset, void *words, int length)
{
    /* Correct for DSC device base address. */
    offset -= DSC_DEVICE_OFFSET;
    int Read;
    return 
        TEST_IO(lseek(DevDsc, offset, SEEK_SET))  &&
        TEST_IO(Read = read(DevDsc, words, length))  &&
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
        TEST_IO(lseek(DevDsc, offset, SEEK_SET))  &&
        TEST_IO(Written = write(DevDsc, words, length)) &&
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
        int Atten = Attenuation;
        if (OldBrillianceApi)
            /* In early versions of the Libera Brilliance FPGA the attenuators
             * were spaced at 0.5dB intervals and the attenuator value was
             * inverted.  In newer versions this is restored to an interface
             * rather more similar to that used by Libera Electron. */
            Atten = ~(Atten << 1);
        memset(AttenuatorWords, Atten, 4);
        AttenuatorWords[1] = 0;
    }
    else
    {
        /* For Libera Electron we split the attenuator value evenly across two
         * attenuators per channel. */
        uint8_t Atten1 = Attenuation / 2;
        uint8_t Atten2 = Attenuation - Atten1;
        uint8_t OneWord[4] = { Atten2, Atten1, Atten2, Atten1 };
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
    uint8_t Template[MAX_SWITCH_SEQUENCE/2];
    for (unsigned int i = 0; i < sizeof(Template); i++)
        Template[i] = SwitchPattern[2*i] | (SwitchPattern[2*i + 1] << 4);

    /* Now prepare the full block before writing it to the DSC device. */
    uint8_t SwitchPatternBlock[DSC_SWITCH_PATTERN_DB];
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


int MaximumAttenuation()
{
    return LiberaBrilliance ? 31 : 62;
}


bool WriteAttenuation(int NewAttenuation)
{
    if (0 <= NewAttenuation  &&  NewAttenuation <= MaximumAttenuation())
    {
        LOCK();
        Attenuation = NewAttenuation;
        MARK_DIRTY(Attenuation);
        UNLOCK();
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
        LOCK();
        /* Copy over the new switch pattern, repeating as necessary to fill
         * up to the standard length.  Only the bottom four bits of each
         * switch are used. */
        for (unsigned int i = 0; i < MAX_SWITCH_SEQUENCE; i += NewLength)
            for (unsigned int j = 0; j < NewLength; j ++)
                SwitchPattern[i+j] = NewSwitches[j] & 0xF;
        MARK_DIRTY(SwitchPattern);
        UNLOCK();
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
    LOCK();
    for (int i = 0; i < BUTTON_COUNT; i ++)
    {
        int Base = ((Switch & 0xF) << 1) | (i << 5);
        RawPhaseComp[Base]   = Array[i][0];
        RawPhaseComp[Base+1] = Array[i][1];
    }
    MARK_DIRTY(PhaseComp);
    UNLOCK();
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
    LOCK();
    for (int j = 0; j < BUTTON_COUNT; j ++)
    {
        int Base = ((Switch & 0xF) << 1) | (j << 6);
        for (int i = 0; i < BUTTON_COUNT; i ++)
            RawSwitchDemux[Base | (i & 1) | ((i & 2) << 4)] = Array[j][i];
    }
    MARK_DIRTY(SwitchDemux);
    UNLOCK();
}



/* Commits all written double-buffer state by switching double buffers. */

bool CommitDscState()
{
    int Buffer;
    return LOCKED(
        /* Pick up which double buffer is currently active. */
        ReadDscWord(DSC_DOUBLE_BUFFER, Buffer)  &&

        /* Write our current (new) state into the current writeable buffer. */
        WriteAttenuatorState(DOUBLE_BUFFER(Buffer, DSC_ATTENUATORS))  &&
        WriteSwitchesState  (DOUBLE_BUFFER(Buffer, DSC_SWITCH_PATTERN))  &&
        WritePhaseState     (DOUBLE_BUFFER(Buffer, DSC_PHASE_COMP))  &&
        WriteDemuxState     (DOUBLE_BUFFER(Buffer, DSC_SWITCH_DEMUX))  &&

        /* Swap the new buffer into place: in effect, an atomic write. */
        WriteDscWord(DSC_DOUBLE_BUFFER, Buffer^1));
}


/* The switch trigger source is controlled by the top bit of the
 * turn-by-turn divider register. */

bool WriteSwitchTriggerSelect(bool ExternalTrigger)
{
    int DividerValue;
    return LOCKED(
        ReadDscWord(DSC_SWITCH_DIVIDER, DividerValue)  &&
        WriteDscWord(DSC_SWITCH_DIVIDER,
            (DividerValue & 0x7FFFFFFF) | (ExternalTrigger << 31)));
}


/* The delay on the switch clock source is programmed into the bottom ten
 * bits of the delay control register. */

bool WriteSwitchTriggerDelay(int Delay)
{
    int DelayControl;
    return LOCKED(
        ReadDscWord(DSC_SWITCH_DELAY, DelayControl)  &&
        WriteDscWord(DSC_SWITCH_DELAY,
            (DelayControl & 0xFFFF0000) | (Delay & 0x3FF)));
}


/* This is not properly part of the DSC interface, but happens to be
 * accessible through the DSC device, as this is the part of the FPGA address
 * space occupied by this register. */

bool WriteInterlockIIR_K(int K)
{
    return LOCKED(WriteDscWord(DSC_INTERLOCK_IIR_K, K));
}



/*****************************************************************************/
/*                                                                           */
/*                           FPGA 2.00+ Features                             */
/*                                                                           */
/*****************************************************************************/


static uint32_t * AverageSumRegisters = NULL;

#ifdef RAW_REGISTER
static bool InitialiseAverageSum()
{
    if (Version2FpgaPresent)
        return TEST_NULL(
            AverageSumRegisters = MapRawRegister(REGISTER_FA_NSUMS));
    else
        return true;
}
#endif


void GetTriggeredAverageSum(int &Sum, int &Samples)
{
    /* We could use the LIBERA_CFG_AVERAGE_SUM configuration call to read
     * this value, but this is one of the unstable configuration numbers
     * (changes between 2.00 and 2.02), and also it can be quite instructive
     * to have the number of samples at the same time.  Thus we read the
     * hardware directly instead. */
    if (AverageSumRegisters == NULL)
    {
        Sum = 0;
        Samples = 0;
    }
    else
    {
        LOCK();
        uint32_t u_samples = AverageSumRegisters[0];
        if (u_samples == 0)
        {
            Samples = 0;
            Sum = 0;
        }
        else
        {
            uint64_t lsw = AverageSumRegisters[1];
            uint64_t msw = AverageSumRegisters[2];
            Sum = (int) (uint32_t) ((lsw + (msw << 32)) / u_samples);
            Samples = u_samples;
        }
        UNLOCK();
    }
}


bool WriteMafSettings(int Offset, int Delay)
{
    return false;
}


static int rem(int a, int b)
{
    int result = a % b;
    return result >= 0 ? result : result + b;
}

#define FA_REG(base, address) \
    ((uint32_t *) ((char *) base + address - FA_OFFSET))

bool WriteSpikeRemovalSettings(
    bool Enable, int AverageWindow, int AverageStop,
    int SpikeStart, int SpikeWindow)
{
    AverageWindow = 1 << AverageWindow;         // Convert to power of 2
    if (DlsFpgaFeatures)
    {
        /* The spike removal settings differ subtly between the i-Tech and DLS
         * FPGAs.  For the DLS FPGA the start point of the average window must
         * be specified, not the stop, and values must be modulo
         * TurnsPerSwitch. */
        AverageStop -= AverageWindow;
        AverageStop = rem(AverageStop, TurnsPerSwitch);
        SpikeStart  = rem(SpikeStart,  TurnsPerSwitch);
    }
    
#if defined(RAW_REGISTER)
    /* Write directly to the hardware in preference to using the driver. */
    uint32_t * FA_area;
    if (!TEST_NULL(FA_area = MapRawRegister(FA_OFFSET)))
        return false;

    LOCK();
    *FA_REG(FA_area, REGISTER_SR_ENABLE)      = Enable;
    *FA_REG(FA_area, REGISTER_SR_AVE_STOP)    = AverageStop;
    *FA_REG(FA_area, REGISTER_SR_AVE_WIN)     = AverageWindow;
    *FA_REG(FA_area, REGISTER_SR_SPIKE_START) = SpikeStart;
    *FA_REG(FA_area, REGISTER_SR_SPIKE_WIN)   = SpikeWindow;
    UNLOCK();
    
    UnmapRawRegister(FA_area);
    return true;
    
#elif defined(__EBPP_H_2)
    int EnableInt = Enable;
    return LOCKED(
        WriteCfgValue(LIBERA_CFG_SR_ENABLE,         EnableInt)  &&
        WriteCfgValue(LIBERA_CFG_SR_AVERAGE_WINDOW, AverageWindow)  &&
        WriteCfgValue(LIBERA_CFG_SR_AVERAGING_STOP, AverageStop)  &&
        WriteCfgValue(LIBERA_CFG_SR_START,          SpikeStart)  &&
        WriteCfgValue(LIBERA_CFG_SR_WINDOW,         SpikeWindow));
    
#else
    return TEST_OK(false);
#endif
}


bool ReadSpikeRemovalBuffer(int Buffer[SPIKE_DEBUG_BUFLEN])
{
    uint32_t * FA_area;
    if (!TEST_NULL(FA_area = MapRawRegister(FA_OFFSET)))
        return false;

    /* Enable spike capture and wait for some waveforms to be captured.
     * If switching is enabled the buffer will be filled within a few
     * microseconds, even on the largest of machines.  So we sleep a
     * little and disable capture before reading out. */
    LOCK();
    *FA_REG(FA_area, REGISTER_SR_DEBUG) = 1;
    usleep(1000);
    *FA_REG(FA_area, REGISTER_SR_DEBUG) = 0;
    memcpy(Buffer, FA_REG(FA_area, REGISTER_SR_BUFFER),
        SPIKE_DEBUG_BUFLEN * sizeof(int));
    UNLOCK();
    
    UnmapRawRegister(FA_area);
    return true;
}



bool WritePmTriggerParameters(
    PM_TRIGGER_SOURCE source,
    int Xlow, int Xhigh, int Ylow, int Yhigh,
    unsigned int overflow_limit, unsigned int overflow_dur)
{
    /* This is another case where the driver API is unstable, so we instead
     * access the registers directly.  Another advantage of doing this this
     * way is that we can use this FPGA feature even without driver support,
     * in particular on 1.46. */

    /* The ADC limit value is rather odd: it's the raw ADC limit in the top 16
     * bits, and the top 16 bits of the ADC limit squared in the bottom. */
    unsigned int overflow_limit_reg =
        ((overflow_limit >> AdcExcessBits) << 16) |
        ((overflow_limit * overflow_limit) >> 16);
    return LOCKED(
        WriteRawRegister(REGISTER_TRIG_DELAY, source << 14, 0x0000C000)  &&
        WriteRawRegister(REGISTER_PM_MINX, Xlow)  &&
        WriteRawRegister(REGISTER_PM_MAXX, Xhigh)  &&
        WriteRawRegister(REGISTER_PM_MINY, Ylow)  &&
        WriteRawRegister(REGISTER_PM_MAXY, Yhigh) &&
        WriteRawRegister(REGISTER_PM_ADC_LIMIT, overflow_limit_reg)  &&
        WriteRawRegister(REGISTER_PM_ADC_TIME,  overflow_dur));
}


bool WriteNotchFilter(int index, NOTCH_FILTER Filter)
{
    uint32_t WriteNotchAddress = 0x1401C018 + 4*index;
    uint32_t * WriteNotch = NULL;
    errno = 0;
    bool Ok = 
        TEST_OK((index & 1) == index)  &&
        TEST_NULL(WriteNotch = MapRawRegister(WriteNotchAddress));
    if (Ok)
    {
        LOCK();
        for (int i = 0; i < 5; i ++)
            *WriteNotch = Filter[i];
        UNLOCK();
        UnmapRawRegister(WriteNotch);
    }
    return Ok;
}



/*****************************************************************************/
/*                                                                           */
/*                      Initialisation and Shutdown                          */
/*                                                                           */
/*****************************************************************************/


#ifdef RAW_REGISTER

/* At present there are two alternative implementations of continuous max ADC
 * reading.  We enable access to the appropriate register here. */
static bool EnableMaxAdc()
{
    if (ItechMaxAdcPresent)
        return TEST_NULL(
            RegisterMaxAdcRaw = MapRawRegister(REGISTER_MAX_ADC_ITECH));
    else if (DlsFpgaFeatures)
        return TEST_NULL(
            RegisterMaxAdcRaw = MapRawRegister(REGISTER_MAX_ADC_DLS));
    else
        /* Not enabled, not a problem. */
        return true;
}

#endif



bool InitialiseHardware(int _TurnsPerSwitch)
{
    TurnsPerSwitch = _TurnsPerSwitch;
    
#ifdef RAW_REGISTER
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;
#endif
    
    /* If the LiberaBrilliance flag is set then the ADC is 16 bits, otherwise
     * we're operating an older Libera with 12 bits.  We actually record and
     * use the excess bits which need to be handled specially. */
    AdcExcessBits = 16 - (LiberaBrilliance ? 16 : 12);
    
    return
        /* Open all the devices we're going to need. */
        TEST_IO(DevCfg   = open("/dev/libera.cfg",   O_RDWR))  &&
        TEST_IO(DevAdc   = open("/dev/libera.adc",   O_RDONLY))  &&
        TEST_IO(DevDsc   = open("/dev/libera.dsc",   O_RDWR | O_SYNC))  &&
        TEST_IO(DevEvent = open("/dev/libera.event", O_RDWR))  &&
        TEST_IO(DevPm    = open("/dev/libera.pm",    O_RDONLY))  &&
        TEST_IO(DevSa    = open("/dev/libera.sa",    O_RDONLY))  &&
        TEST_IO(DevDd    = open("/dev/libera.dd",    O_RDONLY))  &&
#ifdef RAW_REGISTER
        TEST_IO(DevMem   = open("/dev/mem", O_RDWR | O_SYNC))  &&
        EnableMaxAdc()  &&
        InitialiseAverageSum()  &&
#endif
        true;
}
