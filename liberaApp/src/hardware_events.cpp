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

/* CSPI/leventd compatible event receiver implementation. */

#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "libera.h"     // Libera device driver header
#include "ebpp.h"       // ditto

#include "hardware.h"
#include "thread.h"
#include "hardware_events.h"


/* Communicate with leventd and ask to be sent events.  Unfortunately, the
 * damn thing insists on sending us events through signals, so we place
 * ourself in another thread for this to avoid the signals interfering with
 * our normal processing. */

class LEVENTD_HANDLER : public THREAD
{
public:
    LEVENTD_HANDLER(int EventMask, int Pipe) :
        EventMask(EventMask),
        Pipe(Pipe)
    {
        TEST_(sem_init, &Shutdown, 0, 0);
    }

    void Thread()
    {
        /* First set up a signal handler to receive the signals that leventd
         * insists on sending. */
        struct sigaction NewSa, OldSa;
        sigemptyset(&NewSa.sa_mask);
        NewSa.sa_sigaction = LiberaSignal;
        NewSa.sa_flags = SA_SIGINFO;
        if (!TEST_(sigaction, SIGUSR1, &NewSa, &OldSa))
            return;
        
        /* Now post a request to leventd to send us the request mask of
         * events.  This involves posting a two word command to the daemon's
         * request queue. */
        struct
        {
            pid_t pid;
            size_t mask;
        } Request = { getpid(), LIBERA_EVENT_TRIGGET };
        int leventd, written;
        bool RequestOk =
            TEST_IO(leventd, "Unable to open leventd command queue",
                open, "/tmp/leventd.fifo", O_WRONLY | O_NONBLOCK)  &&
            TEST_IO(written, "Unable to write to leventd command queue",
                write, leventd, &Request, sizeof(Request))  &&
            TEST_(close, leventd)  &&
            written == sizeof(Request);
        /* If all is well then report success and then just sit and wait for
         * shutdown: all the real work now happens in the background. */
        if (RequestOk)
        {
            StartupOk();
            TEST_(sem_wait, &Shutdown);
        }

        /* Reset the original signal handler on exit. */
        TEST_(sigaction, SIGUSR1, &OldSa, NULL);
    }

    static LEVENTD_HANDLER * EventHandler;

private:
    /* Signal handler.  Needs to be static, alas. */
    static void LiberaSignal(int Signal, siginfo_t *Siginfo, void*)
    {
        /* Reassemble, as far as possible, the original event type.  The
         * original event id and parameter have been packed together into
         * si_value, so we need to unpack them again before posting on the
         * reconstructed event. */
        libera_event_t Event;
        int Packed = Siginfo->si_value.sival_int;
        Event.id = (Packed >> 16) & 0xFFFF;     // Id in top half
        Event.param = Packed & 0xFFFF;          // Param in bottom half
        int written = write(EventHandler->Pipe, &Event, sizeof(Event));
        /* Difficult to report problems here, actually.  We're inside a
         * signal handler! */
        if (written != sizeof(Event))
            /* Well? */ ;
    }

    void OnTerminate()
    {
        /* Release the main thread: if all is well, it is blocked at this
         * very moment waiting for this shutdown signal. */
        TEST_(sem_post, &Shutdown);
    }

    
    /* Mask of events to be requested from the Libera event system. */
    const int EventMask;
    /* File handle to which event reports will be written. */
    int Pipe;
    /* Semaphore used to signal shutdown of this thread. */
    sem_t Shutdown;
};


LEVENTD_HANDLER * LEVENTD_HANDLER::EventHandler = NULL;

bool StartEventHandler(int EventMask, int Pipe) 
{
    LEVENTD_HANDLER::EventHandler = new LEVENTD_HANDLER(EventMask, Pipe);
    return LEVENTD_HANDLER::EventHandler->StartThread();
}

void StopEventHandler()
{
    if (LEVENTD_HANDLER::EventHandler != NULL)
        LEVENTD_HANDLER::EventHandler->Terminate();
}

