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

/* Implementation of simple thread class. */

#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>

#include "hardware.h"
#include "thread.h"




/*****************************************************************************/
/*                                                                           */
/*                             SEMAPHORE class                               */
/*                                                                           */
/*****************************************************************************/


/* We roll our own semaphore class here because of unresolved (and really
 * rather unexpected) problems with sem_t and in particular with
 * sem_timedwait.  The biggest problem is a very mysterious effect where
 * sem_timedwait returns ETIMEDOUT *immediately*!  Very odd indeed... */

SEMAPHORE::SEMAPHORE(bool InitialReady)
{
    /* Initialise our internal resources. */
    Ready = InitialReady;
    TEST_(pthread_cond_init, &ReadyCondition, NULL);
    TEST_(pthread_mutex_init, &ReadyMutex, NULL);
}

bool SEMAPHORE::Wait(int Seconds)
{
    timeval Now;
    TEST_(gettimeofday, &Now, NULL);
    timespec Timeout;
    Timeout.tv_sec  = Now.tv_sec + Seconds;
    Timeout.tv_nsec = Now.tv_usec * 1000;
    
    Lock();
    int ReturnCode = 0;
    while (!Ready  &&  ReturnCode == 0)
        ReturnCode = pthread_cond_timedwait(
            &ReadyCondition, &ReadyMutex, &Timeout);
    Ready = false;
    UnLock();

    return ReturnCode == 0;
}

bool SEMAPHORE::Signal()
{
    Lock();
    bool OldReady = Ready;
    Ready = true;
    TEST_(pthread_cond_signal, &ReadyCondition);
    UnLock();
    return OldReady;
}

void SEMAPHORE::Lock()
{
    TEST_(pthread_mutex_lock, &ReadyMutex);
}

void SEMAPHORE::UnLock()
{
    TEST_(pthread_mutex_unlock, &ReadyMutex);
}



/*****************************************************************************/
/*                                                                           */
/*                               THREAD class                                */
/*                                                                           */
/*****************************************************************************/



THREAD::THREAD(const char * Name) :
    Name(Name)
{
    TEST_(sem_init, &ThreadStatusSemaphore, 0, 0);
}


/* If the thread was successfully started then wait for it to report back
 * before returning the status flag. */

bool THREAD::StartThread()
{
    ThreadRunning = true;
    ThreadOkFlag = false;
    /* Start the thread and then wait for it to report back. */
    if (TEST_(
            pthread_create, &ThreadId, NULL, StaticThread, this))
        TEST_(sem_wait, &ThreadStatusSemaphore);
    /* At this point we know that the thread is only running if StartupOk()
     * was called, so ThreadOkFlag is a good proxy for the thread's state. */
    return ThreadOkFlag;
}

void THREAD::Terminate()
{
    if (ThreadOkFlag)
    {
        /* Let the thread know that it should be stopping now. */
        ThreadRunning = false;
        OnTerminate();
        /* Wait for the thread to finish (hope we don't get stuck here!) */
        TEST_(pthread_join, ThreadId, NULL);
        /* Call any post shutdown processing: poor man's pthread_cancel
         * hooks! */
        ThreadShutdown();
    }
}


void THREAD::OnTerminate()
{
    /* The default terminate action is to cancel the thread directly. */
    TEST_(pthread_cancel, ThreadId);
}


/* Record startup status and signal possibly waiting caller. */

void THREAD::StartupOk()
{
    ThreadOkFlag = true;
    TEST_(sem_post, &ThreadStatusSemaphore);
}


/* Simple hook to transfer the thread into a full member function. */

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
    TEST_(sem_post, &ThreadStatusSemaphore);
}



LOCKED_THREAD::LOCKED_THREAD(const char * Name) :
    THREAD(Name)
{
    /* Create the thread synchronisation primitives.  Note that these are
     * never destroyed because this class (instance) is never destroyed ...
     * because EPICS doesn't support any kind of restart, there's no point in
     * doing this anywhere else! */
    TEST_(pthread_cond_init, &Condition, NULL);
    TEST_(pthread_mutex_init, &Mutex, NULL);
}


/* Some units of time to avoid confusion when counting zeros! */
#define S_NS    1000000000      // 1s in ns
#define MS_NS   1000000         // 1ms in ns
#define MS_S    1000            // 1s in ms

bool LOCKED_THREAD::WaitFor(int milliseconds)
{
    struct timespec target;
    if (!TEST_(clock_gettime, CLOCK_REALTIME, &target))
        return false;

    int seconds = milliseconds / MS_S;
    milliseconds -= seconds * MS_S;
    target.tv_sec += seconds;
    target.tv_nsec += milliseconds * MS_NS;
    if (target.tv_nsec > S_NS)
    {
        target.tv_sec += 1;
        target.tv_nsec -= S_NS;
    }
    
    return WaitUntil(&target);
}


bool LOCKED_THREAD::WaitUntil(const struct timespec *target)
{
    int rc = pthread_cond_timedwait(&Condition, &Mutex, target);
    if (rc == 0)
        return true;
    else
    {
        if (rc != ETIMEDOUT)
            perror("pthread_cond_timedwait");
        return false;
    }
}
