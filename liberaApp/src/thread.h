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

#include <semaphore.h>
#include <pthread.h>


/* Class to implement a simple thread. */

class THREAD
{
public:
    THREAD(const char * Name);

    /* Starts the thread and waits for it to report back on its initial
     * startup status.  If the thread reports back with StartupOk then true
     * is returned; if the thread returns first then false is returned. */
    bool StartThread();

    /* Synchronised termination of thread.  Relies on thread either
     * responding to OnTerminate() method or polling Running() method. */
    void Terminate();

protected:
    /* This is the thread itself. */
    virtual void Thread() = 0;

    /* Thread termination handler: called during thread shutdown (if
     * pthread_cancel was used). */
    virtual void ThreadShutdown() { }

    /* This routine will be called to request termination of the thread.  By
     * default we use pthread_cancel(). */
    virtual void OnTerminate();

    /* This routine must be called by the thread when it has finished its
     * initialisation so that the startup status can be reported back to the
     * caller.  If this is not called then ThreadOk() will block.
     *     The model is straightforward: either the thread reports that it
     * started ok and then continues operation, or else it terminates early
     * without reporting success. */
    void StartupOk();

    /* This can be polled to ensure termination of the thread. */
    bool Running() { return ThreadRunning; }

    /* Sends a signal to this thread. */
    void Kill(int sig);

    
private:
    static void * StaticThread(void * Context);
    void ThreadInit();
    
    bool ThreadRunning;         // Used to request thread termination
    /* Thread id of this thread. */
    pthread_t ThreadId;
    /* These two work together once the thread has started: once the thread
     * knows its status it signals the semaphore after ensuring that the ok
     * flag is set appropriate. */
    bool ThreadOkFlag;
    sem_t ThreadStatusSemaphore;
    /* Thread identification for debuggin. */
    const char * Name;
};


/* A class to implement binary semaphores with timeout. */

class SEMAPHORE
{
public:
    SEMAPHORE(bool InitialReady);

    /* Unconditional wait. */
    void Wait();
    /* Wait for semaphore to be ready and consumes the ready flag. */
    bool WaitFor(int milliseconds);
    bool WaitUntil(const struct timespec &target);

    /* Signal that semaphore is ready: returns the previous state of the
     * ready flag. */
    bool Signal();
    
private:
    void Lock();
    static void Unlock(void *arg);

    bool Ready;
    pthread_cond_t ReadyCondition;
    pthread_mutex_t ReadyMutex;
};


/* Simple locking support. */

class LOCKED
{
public:
    LOCKED();
protected:
    void Lock()   { TEST_0(pthread_mutex_lock(&Mutex)); }
    static void Unlock(void *arg);
private:
    pthread_mutex_t Mutex;
};

/* Helper macros for LOCKED class. */
#define THREAD_LOCK(self) \
    (self)->Lock(); \
    pthread_cleanup_push((self)->Unlock, (LOCKED *) self)
#define THREAD_UNLOCK() \
    pthread_cleanup_pop(true)


/* A thread class with mutex locking already built in. */

class LOCKED_THREAD : public THREAD, public LOCKED
{
public:
    LOCKED_THREAD(const char * Name) : THREAD(Name), LOCKED() {}
};


/* On older revisions of the Libera driver and the pthread library we can't
 * use pthread_cancel() to close our threads, because it kills the driver!
 * Fortunately (I hope...) with the new threading library and new kernel
 * environment that comes with the ARM EABI this problem will have gone away.
 * We define this symbol to check whether to override OnTerminate. */
#ifndef __ARM_EABI__
#define UNSAFE_PTHREAD_CANCEL
#endif
