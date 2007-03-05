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

#include "hardware.h"



#define CSPI_(command, args...) \
    ( { int error = (command)(args); \
        if (error != CSPI_OK) \
            printf("CSPI error in %s: %s\n", #command, cspi_strerror(error)); \
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


bool WriteSwitchState(CSPI_SWITCHMODE Switches)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.switches = Switches;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_SWITCH);
}


bool WriteAgcMode(CSPI_AGCMODE AgcMode)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.agc = AgcMode;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_AGC);
}


bool WriteDscMode(CSPI_DSCMODE DscMode)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.dsc = DscMode;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_DSC);
}


bool ReadSwitches(int &Switches)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_SWITCH);
    if (Ok)
        Switches = EnvParams.switches;
    return Ok;
}


/* The interface to the attenuators is somewhat indirect in this version of
 * the CSPI.  The file /opt/dsc/gain.conf defines a mapping from "gain level"
 * to attenuation, with level=17 mapped to attenuation=62, down to level=-45
 * mapped to attenuation=0.
 *    To simplify things we reverse this mapping again, using the formula:
 *      attenuation - level = 45.
 *    To further confuse things, the appropriate EnvParams field (where
 * "level" is configured) is named gain below. */

#define ZEROdBmATT 62

bool WriteAttenuation(int Attenuation)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.gain = Attenuation - ZEROdBmATT;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_GAIN);
}

bool ReadAttenuation(int &Attenuation)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_GAIN);
    if (Ok)
        Attenuation = EnvParams.gain + ZEROdBmATT;
    return Ok;
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


bool ReadHealth(cspi_health_t &Health)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_HEALTH);
    if (Ok)
        Health = EnvParams.health;
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
    return
        CSPI_(cspi_read_ex, CspiConAdc, &Data, 1024, &Read, NULL)  &&
        Read == 1024;
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
    OsPageSize = getpagesize();
    OsPageMask = OsPageSize - 1;
    
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
        /* Open /dev/mem for register access. */
        TEST_IO(DevMem, "Unable to open /dev/mem",
            open, "/dev/mem", O_RDWR | O_SYNC);
}
