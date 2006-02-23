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

/* Implementation of simple thread class. */

#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

#include "hardware.h"
#include "thread.h"


THREAD::THREAD()
{
    TEST_(sem_init, &ThreadStatus, 0, 0);
}


/* If the thread was successfully started then wait for it to report back
 * before returning the status flag. */

bool THREAD::StartThread()
{
    ThreadRunning = true;
    ThreadOkFlag = false;
    ThreadStarted = TEST_(
        pthread_create, &ThreadId, NULL, StaticThread, this);
    if (ThreadStarted)
        TEST_(sem_wait, &ThreadStatus);
//    printf("StartThread returns %d\n", ThreadOkFlag);
    return ThreadOkFlag;
}

void THREAD::Terminate()
{
    if (ThreadStarted)
    {
        /* Let the thread know that it should be stopping now. */
        ThreadRunning = false;
        OnTerminate();
        
        /* It would be good to signal or otherwise interrupt the thread
         * at this point ... this is less than satisfactory.  The problem
         * is that if ReadSlowAcquisition blocks for whatever reason then
         * we may never get past this point. */
//            if (ThreadPid != -1)  kill(ThreadPid, SIGQUIT);
        TEST_(pthread_join, ThreadId, NULL);
    }
}


/* Record startup status and signal possibly waiting caller. */

void THREAD::StartupOk()
{
    ThreadOkFlag = true;
    TEST_(sem_post, &ThreadStatus);
}


/* Simple hook to bring the thread into a full member function. */

void * THREAD::StaticThread(void * Context)
{
    ((THREAD *) Context)->ThreadInit();
    return NULL;
}


/* Hook for any work that needs to be done on startup in the new thread. */

void THREAD::ThreadInit()
{
    Thread();
    /* On thread termination ensure the thread status condition is signalled:
     * if we come here without ThreadOk() being called then the thread failed
     * on startup. */
    TEST_(sem_post, &ThreadStatus);
}
