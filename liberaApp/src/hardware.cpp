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


/* Libera device interface reimplemented through CSPI. */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/unistd.h>

#define EBPP
#include "cspi.h"

#include "thread.h"
#include "hardware.h"
#include "support.h"


/* Disable PM support for the moment: it seems particularly broken! */
#undef ENABLE_PM


#define CSPI_(command, args...) \
    ( { int error = (command)(args); \
        if (error != CSPI_OK) \
            printf("CSPI error in %s: %s\n", #command, cspi_strerror(error)); \
        error == CSPI_OK; } )


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



/*****************************************************************************/
/*                                                                           */
/*                      Miscellaneous Support Routines.                      */
/*                                                                           */
/*****************************************************************************/


/* This array translates switch positions into button permutations. */

static const PERMUTATION PermutationLookup[16] =
{
    { 0, 1, 2, 3 },  { 0, 2, 1, 3 },  { 3, 1, 2, 0 },  { 3, 2, 1, 0 },
    { 0, 1, 3, 2 },  { 0, 2, 3, 1 },  { 3, 1, 0, 2 },  { 3, 2, 0, 1 },
    { 1, 0, 2, 3 },  { 2, 0, 1, 3 },  { 1, 3, 2, 0 },  { 2, 3, 1, 0 },
    { 1, 0, 3, 2 },  { 2, 0, 3, 1 },  { 1, 3, 0, 2 },  { 2, 3, 0, 1 }
};


/* We think of the attenuators in the ATTENUATORS array as setting values for
 * the first and second stages of channels 1 to 4, in the order
 *      ch1 1st, ch1 2nd, ch2 1st, ..., ch4 2nd . */

bool ReadAttenuators(ATTENUATORS Attenuators)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_ATTN);
    if (Ok)
        for (int i = 0; i < CSPI_MAXATTN; i ++)
            Attenuators[i] = EnvParams.attn[i];
    return Ok;
}

bool WriteAttenuators(const ATTENUATORS Attenuators)
{
    CSPI_ENVPARAMS EnvParams;
    for (int i = 0; i < CSPI_MAXATTN; i ++)
        EnvParams.attn[i] = Attenuators[i];
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_ATTN);
}

bool ReadSwitches(int &Switches)
{
    CSPI_ENVPARAMS EnvParams;
    bool Ok = CSPI_(cspi_getenvparam, CspiEnv, &EnvParams, CSPI_ENV_SWITCH);
    if (Ok)
        Switches = EnvParams.switches;
    return Ok;
}

bool WriteSwitches(int Switches)
{
    CSPI_ENVPARAMS EnvParams;
    EnvParams.switches = Switches;
    return CSPI_(cspi_setenvparam, CspiEnv, &EnvParams, CSPI_ENV_SWITCH);
}



/*****************************************************************************/
/*                                                                           */
/*                   Reading waveform data from the FPGA.                    */
/*                                                                           */
/*****************************************************************************/


size_t ReadWaveform(int Decimation, size_t WaveformLength, LIBERA_ROW * Data)
{
    CSPI_CONPARAMS_DD ConParams;
    ConParams.dec = Decimation;
    unsigned long long Offset = 0;
    size_t Read;
    bool Ok =
        // Set the decimation mode
        CSPI_(cspi_setconparam, CspiConDd,
            (CSPI_CONPARAMS *)&ConParams, CSPI_CON_DEC)  &&
        // Seek to the trigger point
        CSPI_(cspi_seek, CspiConDd, &Offset, CSPI_SEEK_TR)  &&
        // Read the data
        CSPI_(cspi_read_ex, CspiConDd, Data, WaveformLength, &Read, NULL);

    if (Ok)
        return Read;
    else
        return 0;
}


size_t ReadPostmortem(size_t WaveformLength, LIBERA_ROW * Data)
{
    size_t Read;
    if (CSPI_(cspi_read_ex, CspiConPm, Data, 16384, &Read, NULL))
        return Read;
    else
        return 0;
}


bool ReadAdcWaveform(ADC_DATA &Data)
{
    int Switches;
    size_t Read;
    bool Ok =
        ReadSwitches(Switches)  &&
        CSPI_(cspi_read_ex, CspiConAdc, &Data.Rows, 1024, &Read, NULL)  &&
        Read == 1024;
    if (Ok)
        memcpy(Data.Permutation, PermutationLookup[Switches],
            sizeof(Data.Permutation));
    return Ok;
}


bool ReadSlowAcquisition(SA_DATA &SaData)
{
    CSPI_SA_ATOM Result;
    if (CSPI_(cspi_get, CspiConSa, &Result))
    {
        SaData.A = Result.Va;
        SaData.B = Result.Vb;
        SaData.C = Result.Vc;
        SaData.D = Result.Vd;
        return true;
    }
    else
        return false;
}



/*****************************************************************************/
/*                                                                           */
/*                        Direct Event Connection                            */
/*                                                                           */
/*****************************************************************************/

/* Libera low level device, used to received event notification. */
static int libera_event = -1;
static int event_source = -1;

/* Reads one event from the event queue and decode it into an event id and
 * its associated parameter information.  The caller should empty the queue
 * by calling this routine until it returns false before processing any
 * events. */

bool ReadOneEvent(HARDWARE_EVENT_ID &Id, int &Param)
{
    libera_event_t event;
    ssize_t bytes_read = read(libera_event, &event, sizeof(event));
    if (bytes_read == 0  ||  (bytes_read == -1  &&  errno == EAGAIN))
        /* Nothing to read this time: the queue is empty. */
        return false;
    else if (bytes_read > 0)
    {
        if (bytes_read < (ssize_t) sizeof(event))
        {
            /* Oops.  Half an event?  This is going to be trouble!  We have
             * no good way to recover from this.  We should, in principle,
             * keep on trying to read until a full event arrives...
             *     Fortunately this never seems to happen. */
            printf("Incomplete event read: %d bytes read", bytes_read);
            return false;
        }
        else
        {
            /* Good.  Return the event. */
            Id = (HARDWARE_EVENT_ID) event.id;
            Param = event.param;
            return true;
        }
    }
    else
    {
        perror("Error reading event queue");
        return false;
    }
}



/* This thread exists purely to receive the leventd signals.  I'm not willing
 * to subject my main thread to signals in case they have unlookedfor and
 * disagreeable side effect (mainly, code out of my control which does not
 * properly check for EINTR or unresumable operations). */

class LEVENTD_THREAD : public THREAD
{
public:
    LEVENTD_THREAD()
    {
        TEST_(sem_init, &Shutdown, 0, 0);
    }
    
private:
    void Thread()
    {
        CSPIHCON EventSource;
        CSPI_CONPARAMS ConParams;
        ConParams.event_mask = CSPI_EVENT_TRIGGET;
#ifdef ENABLE_PM
        ConParams.event_mask |= LIBERA_EVENT_PM;
#endif
        ConParams.handler = CspiSignal;
        bool Ok =
            CSPI_(cspi_allochandle, CSPI_HANDLE_CON, CspiEnv, &EventSource)  &&
            CSPI_(cspi_setconparam, EventSource, &ConParams,
                CSPI_CON_EVENTMASK | CSPI_CON_HANDLER);
        
        /* If all is well then report success and then just sit and wait for
         * shutdown: all the real work now happens in the background. */
        if (Ok)
        {
            StartupOk();
            TEST_(sem_wait, &Shutdown);
        }

        CSPI_(cspi_freehandle, CSPI_HANDLE_CON, EventSource);
    }

    /* Signal handler.  Needs to be static, alas. */
    static int CspiSignal(CSPI_EVENT *Event)
    {
        int written = write(event_source, &Event->hdr, sizeof(Event->hdr));
        /* Difficult to report problems here, actually.  We're inside a
         * signal handler! */
        if (written != sizeof(Event))
            /* Well? */ ;
        return 1;
    }

    void OnTerminate()
    {
        TEST_(sem_post, &Shutdown);
    }

    sem_t Shutdown;
};


static LEVENTD_THREAD * LeventdThread = NULL;

/* To be called on initialisation to enable delivery of events. */

static bool OpenEventStream()
{
    int Pipes[2];
    if (TEST_(pipe, Pipes)  &&
        TEST_(fcntl, Pipes[0], F_SETFL, O_NONBLOCK))
    {
        libera_event = Pipes[0];
        event_source = Pipes[1];
        LeventdThread = new LEVENTD_THREAD();
        return LeventdThread->StartThread();
    }
    else
        return false;
}


/* Because the event consumer needs to use select() to wait for event
 * delivery we need to break encapsulation here and expose the actual handle
 * used to receive events. */

int EventSelector()
{
    return libera_event;
}


/* Called on termination to cancel event delivery. */

static void CloseEventStream()
{
    if (LeventdThread != NULL)
        LeventdThread->Terminate();
    if (event_source != -1)  close(event_source);
    if (libera_event != -1)  close(libera_event);
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
    /* A default attenuator setting of 20/20 (40 dB total) ensures a safe
     * starting point on initialisation. */
    static const ATTENUATORS DefaultAttenuators =
        { 20, 20, 20, 20, 20, 20, 20, 20 };

    CSPI_LIBPARAMS LibParams;
    LibParams.superuser = 1;    // Allow us to change settings!
    return
        /* First ensure that the library allows us to change settings! */
        CSPI_(cspi_setlibparam, &LibParams, CSPI_LIB_SUPERUSER)  &&
        /* Open the CSPI environment and then open CSPI handles for each of
         * the data channels we need. */
        CSPI_(cspi_allochandle, CSPI_HANDLE_ENV, 0, &CspiEnv)  &&
        InitialiseConnection(CspiConAdc, CSPI_MODE_ADC)  &&
        InitialiseConnection(CspiConDd, CSPI_MODE_DD)  &&
        InitialiseConnection(CspiConSa, CSPI_MODE_SA)  &&
#ifdef ENABLE_PM
        InitialiseConnection(CspiConPm, CSPI_MODE_PM)  &&
#endif
        /* Set the attenuators and switches into a sensible default state. */
        WriteAttenuators(DefaultAttenuators)  &&
        WriteSwitches(0)  &&
        /* Finally start receiving events. */
        OpenEventStream();
}


static void TerminateConnection(CSPIHCON Connection)
{
    if (Connection != NULL)
    {
        CSPI_(cspi_disconnect, Connection);
        CSPI_(cspi_freehandle, CSPI_HANDLE_CON, Connection);
    }
}

/* To be called on shutdown to release all connections to Libera. */
void TerminateHardware()
{
    CloseEventStream();
    TerminateConnection(CspiConPm);
    TerminateConnection(CspiConDd);
    TerminateConnection(CspiConSa);
    TerminateConnection(CspiConAdc);
    CSPI_(cspi_freehandle, CSPI_HANDLE_ENV, CspiEnv);
}
