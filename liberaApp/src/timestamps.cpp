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

/* Timestamps and synchronisation. */


#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/reboot.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>
#include <time.h>

#include <dbFldTypes.h>

#include "libera_pll.h"

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "events.h"
#include "thread.h"
#include "trigger.h"

#include "timestamps.h"




/*****************************************************************************/
/*                                                                           */
/*                              LMTD Interface                               */
/*                                                                           */
/*****************************************************************************/


/* Commands to the LMTD are transmitted through this file handle. */
FILE * LmtdCommandFile = NULL;


/* LMTD configuration: each of these is written to the LMTD. */
static int SampleClockDetune = 0;   // CK:DETUNE - frequency offset
static int IfClockDetune = 0;       // CK:IFOFF - IF offset ("double detune")
static int PhaseOffset = 0;         // CK:PHASE - phase offset



/* Sends a command to the LMTD daemon. */

static void SendLmtdCommand(const char * format, ...)
{
    va_list args;
    va_start(args, format);
    vfprintf(LmtdCommandFile, format, args);
    fflush(LmtdCommandFile);
}



/* Manage the configured LMTD state by sending the appropropriate commands to
 * the LMTD. */

static void UpdateLmtdState()
{
    SendLmtdCommand("o%d\n", SampleClockDetune);
    SendLmtdCommand("n%d\n", IfClockDetune + SampleClockDetune);
    SendLmtdCommand("p%d\n", PhaseOffset);
}



/* This thread manages the lmtd state reporting thread.  All status reported
 * from the LMTD is read and converted into updating EPICS PVs. */

class LMTD_MONITOR : public THREAD
{
public:
    LMTD_MONITOR()
    {
        Publish_mbbi("CK:LMTD", LmtdState);
        Publish_longin("CK:DAC", DacSetting);
        Publish_longin("CK:PHASE_E", PhaseError);
        Publish_longin("CK:FREQ_E", FrequencyError);
        Interlock.Publish("CK");
    }
    
private:
    /* Decodes a single status line read from the LMTD and ensures that
     * updates are reported as appropriate. */
    void ProcessStatusLine(const char * Line)
    {
        int Scanned = sscanf(Line, "%d %d %d %d\n",
            &LmtdState, &FrequencyError, &PhaseError, &DacSetting);
        if (Scanned != 4)
            printf("Error scanning lmtd status line \"%s\"\n", Line);
    }
    
    void Thread()
    {
        FILE * LmtdStatus = fopen("/tmp/lmtd.status", "r");
        if (LmtdStatus == NULL)
            return;
        StartupOk();

        char * Line = NULL;
        size_t LineLength = 0;
        while (Running())
        {
            if (getline(&Line, &LineLength, LmtdStatus) == -1)
                perror("Error reading lmtd status");
            else
            {
                Interlock.Wait();
                ProcessStatusLine(Line);
                Interlock.Ready();
            }
        }
        fclose(LmtdStatus);
    }


    /* LMTD status variables. */
    int LmtdState;
    int DacSetting;
    int PhaseError;
    int FrequencyError;

    INTERLOCK Interlock;
};


static LMTD_MONITOR * LmtdMonitorThread = NULL;

static bool InitialiseLmtdState()
{
    LmtdCommandFile = fopen(LMTD_COMMAND_FIFO, "w");
    if (LmtdCommandFile == NULL)
    {
        perror("Unable to open LMTD command fifo");
        return false;
    }

    PUBLISH_CONFIGURATION("CK:DETUNE", longout,
        SampleClockDetune, UpdateLmtdState);
    PUBLISH_CONFIGURATION("CK:IFOFF", longout,
        IfClockDetune, UpdateLmtdState);
    PUBLISH_CONFIGURATION("CK:PHASE", longout,
        PhaseOffset, UpdateLmtdState);
    
    UpdateLmtdState();
    
    LmtdMonitorThread = new LMTD_MONITOR;
    return LmtdMonitorThread->StartThread();
}


static void TerminateLmtdState()
{
    if (LmtdCommandFile != NULL)
        fclose(LmtdCommandFile);
    if (LmtdMonitorThread != NULL)
        LmtdMonitorThread->Terminate();
}



/*****************************************************************************/
/*                                                                           */
/*                           Clock Synchronisation                           */
/*                                                                           */
/*****************************************************************************/


/* This class manages system clock synchronisation. */

class SYNCHRONISE_CLOCKS : public THREAD, I_EVENT
{
public:
    SYNCHRONISE_CLOCKS() :
        SyncState(false)
    {
        RunningState = WAITING;
        
        PUBLISH_ACTION("CK:SYNCSC", SynchroniseSystemClock);
        PUBLISH_ACTION("CK:SYNCMC", SynchroniseMachineClock);
        Publish_bi("CK:SYNCSTATE", SyncState);
        
        RegisterTriggerSetEvent(*this, 100);

        /* Create the thread synchronisation primitives.  Note that these are
         * never destroyed because this class (instance) is never destroyed
         * ... because EPICS doesn't support any kind of restart, there's no
         * point in doing this anywhere else! */
        TEST_(pthread_cond_init, &Condition, NULL);
        TEST_(pthread_mutex_init, &Mutex, NULL);
    }
    
    
private:
    /* This thread runs until shutdown.  Normally it has nothing to do, but
     * during clock synchronisation it sets the clock so that clock
     * synchronisation is exact. */
    void Thread()
    {
        StartupOk();

        Lock();
        while (Running())
        {
            /* Wait for synchronisation request. */
            Wait();
            
            while (RunningState == SYNCHRONISING)
            {
                Unlock();
                
                struct timespec NewTime;
                /* Ensure that if the trigger occurs within the next second
                 * then we will correctly pick up the current time. */
                TEST_(clock_gettime, CLOCK_REALTIME, & NewTime);
                long Offset = NewTime.tv_nsec;

                /* The trigger will occur on the second, so program the clock
                 * to expect it on the next whole second. */
                NewTime.tv_sec += 1;
                NewTime.tv_nsec = 0;
                SetSystemClockTime(NewTime);

                /* Now wait until 200ms past this new second.  This gives us
                 * enough time to receive the trigger, if it's coming,
                 * allowing for quite a large NTP time error, and gives us
                 * plenty of time to set up for the next trigger. */
                unsigned int Delay = 1200000 - Offset / 1000;
                usleep(Delay);

                Lock();
            }

        }
        Unlock();
    }


    /* Called during shutdown to request orderly thread termination. */
    void OnTerminate()
    {
        Lock();
        RunningState = TERMINATING;
        Signal();
        Unlock();
    }

    /* This is called in response to processing the CK:SYNCSC record, to tell
     * us that the next trigger will be a system clock synchronisation
     * trigger.  We wake up the main thread to do this work for us. */
    bool SynchroniseSystemClock()
    {
        Lock();
        if (RunningState == WAITING)
            RunningState = SYNCHRONISING;
        SyncState.Write(true);
        Signal();
        Unlock();
        return true;
    }


    /* This is called in response to processing the CK:SYNCMC record: the
     * next trigger is a machine clock synchronisation trigger.  In response
     * we need to let LMTD know that a sync is about to happen. */
    bool SynchroniseMachineClock()
    {
        SetMachineClockTime();
        return true;
    }


    /* This is called when the TRIGSET event is received: this informs us
     * that the clock setting trigger has been received and so clock setting
     * is complete. */
    void OnEvent()
    {
        Lock();
        if (RunningState == SYNCHRONISING)
            RunningState = WAITING;
        SyncState.Write(false);
        Signal();
        Unlock();
    }
    

    /* Wrappers for synchronisation. */
    void Lock()   { TEST_(pthread_mutex_lock,   &Mutex); }
    void Unlock() { TEST_(pthread_mutex_unlock, &Mutex); }
    void Signal() { TEST_(pthread_cond_signal,  &Condition); }
    void Wait()   { TEST_(pthread_cond_wait,    &Condition, &Mutex); }
    
    
    TRIGGER SyncState;

    enum RUNNING_STATE { WAITING, SYNCHRONISING, TERMINATING };
    RUNNING_STATE RunningState;
    
    pthread_cond_t Condition;
    pthread_mutex_t Mutex;
};



static SYNCHRONISE_CLOCKS * SynchroniseThread = NULL;

static bool InitialiseSynchroniseClocks()
{
    SynchroniseThread = new SYNCHRONISE_CLOCKS;
    return SynchroniseThread->StartThread();
}

static void TerminateSynchroniseClocks()
{
    if (SynchroniseThread != NULL)
        SynchroniseThread->Terminate();
}




/*****************************************************************************/
/*                                                                           */
/*                        Reboot and Restart Support                         */
/*                                                                           */
/*****************************************************************************/

/* This code is somewhat out of place here. */


/* Fairly generic routine to start a new detached process in a clean
 * environment. */

static void DetachProcess(const char *Process, char *const argv[])
{
    /* We fork twice to avoid leaving "zombie" processes behind.  These are
     * harmless enough, but annoying.  The double-fork is a simple enough
     * trick. */
    pid_t MiddlePid = fork();
    if (MiddlePid == -1)
        perror("Unable to fork");
    else if (MiddlePid == 0)
    {
        pid_t NewPid = fork();
        if (NewPid == -1)
            perror("Unable to fork");
        else if (NewPid == 0)
        {
            /* This is the new doubly forked process.  We still need to make
             * an effort to clean up the environment before letting the new
             * image have it. */

            /* Set a sensible home directory. */
            TEST_(chdir, "/");

//             /* Restore default signal handling.  I'd like to hope this step
//              * isn't needed. */
//             for (int i = 0; i < NSIG; i ++)
//                 signal(i, SIG_DFL);

            /* Enable all signals. */
            sigset_t sigset;
            TEST_(sigfillset, &sigset);
            TEST_(sigprocmask, SIG_UNBLOCK, &sigset, NULL);

            /* Close all the open file handles.  The new lmtd gets into trouble
             * if we don't do this. */
            for (int i = 0; i < sysconf(_SC_OPEN_MAX); i ++)
                close(i);

            /* Finally we can actually exec the new process... */
            char * envp[] = { NULL };
            TEST_(execve, Process, argv, envp);
        }
        else
            /* The middle process simply exits without further ceremony.  The
             * idea here is that this orphans the new process, which means
             * that the parent process doesn't have to wait() for it, and so
             * it won't generate a zombie when it exits. */
            _exit(0);
    }
    else
    {
        /* Wait for the middle process to finish. */
        if (waitpid(MiddlePid, NULL, 0) == -1)
            perror("Error waiting for middle process");
    }
}


static bool Dummy;

static void DoReboot()
{
    char * Args[] = { "/sbin/reboot", NULL };
    DetachProcess(Args[0], Args);
}


static void DoRestart()
{
    char * Args[] = { "/etc/init.d/epics", "restart", NULL };
    DetachProcess(Args[0], Args);
}



/****************************************************************************/
/*                                                                          */


bool InitialiseTimestamps()
{
    if (!InitialiseSynchroniseClocks())
        return false;
    
    InitialiseLmtdState();

    /* Also a couple of rather miscellanous PVs that could go anywhere.  They
     * currently live here so that they can share the DetachProcess() api. */
    PUBLISH_FUNCTION_OUT(bo, "REBOOT",  Dummy, DoReboot);
    PUBLISH_FUNCTION_OUT(bo, "RESTART", Dummy, DoRestart);

    return true;
}


void TerminateTimestamps()
{
    TerminateSynchroniseClocks();
    TerminateLmtdState();
}
