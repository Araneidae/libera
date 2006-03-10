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

#include <semaphore.h>
#include <pthread.h>

/* Class to implement a simple thread. */

class THREAD
{
public:
    THREAD();

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

    /* This routine will be called to request termination of the thread. */
    virtual void OnTerminate() { }

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
    
    bool ThreadStarted;         // Record of startup ok
    bool ThreadRunning;         // Used to request thread termination
    bool ThreadOkFlag;          // Thread startup status
    pthread_t ThreadId;         // 
    sem_t ThreadStatus;         // Semaphore signalled when thread resolved
};
