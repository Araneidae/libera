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


/* A thread class with locking and event notification already built in. */

class LOCKED_THREAD : public THREAD
{
public:
    LOCKED_THREAD(const char * Name);

protected:
    void Lock()   { TEST_0(pthread_mutex_lock,   &Mutex); }
    void Unlock() { TEST_0(pthread_mutex_unlock, &Mutex); }
    void Signal() { TEST_0(pthread_cond_signal,  &Condition); }
    void Wait()   { TEST_0(pthread_cond_wait,    &Condition, &Mutex); }

    /* Waits for at least the specified number of milliseconds or until the
     * specified time before timing out.  Returns true if the wait was
     * interrupted by a signal, false if a timeout occurred. */
    bool WaitFor(int milliseconds);
    bool WaitUntil(const struct timespec &target);
    
private:
    pthread_cond_t Condition;
    pthread_mutex_t Mutex;
};


/* A class to implement binary semaphores with timeout. */

class SEMAPHORE
{
public:
    SEMAPHORE(bool InitialReady);

    /* Wait for semaphore to be ready and consumes the ready flag.  Returns
     * false on a timeout. */
    bool Wait(int Seconds);

    /* Signal that semaphore is ready: returns the previous state of the
     * ready flag. */
    bool Signal();
    
private:
    void Lock();
    void UnLock();

    bool Ready;
    pthread_cond_t ReadyCondition;
    pthread_mutex_t ReadyMutex;
};
