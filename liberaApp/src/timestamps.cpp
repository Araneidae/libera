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
/*                             Tuning Management                             */
/*                                                                           */
/*****************************************************************************/

#define LMTD_STATE_FILE "/tmp/lmtd.state"
#define LMTD_PID_FILE   "/var/run/lmtd.pid"


/* Sample clock detune state: we toggle the frequency offset of the sample
 * clock between two modes, tuned and detuned.
 *    This is a temporary hack until the detune is built in. */
static int DetuneFactor = 400;
enum LMTD_STATE
{
    LMTD_TUNED = 0,
    LMTD_DETUNED = 1,
    LMTD_DOUBLE_DETUNE = 2
};
static int DetuneState = LMTD_TUNED;



/* The following magic numbers are required for controlling the LMTD and
 * computing the intermediate frequency scaler. */

static int LmtdPrescale;
static int SamplesPerTurn;
static int BunchesPerTurn;


/* Tries the given test repeatedly until it succeeds.  We try every 10ms for
 * up to a second before giving up. */
#define TRY_WITH_TIMEOUT(TEST) \
    ( { \
        bool __ok = (TEST); \
        for (int __i = 0; !__ok && __i < 100; __i ++) \
        { \
            usleep(10000); \
            __ok = (TEST); \
        } \
        __ok; \
    } )


static bool ReadLmtdPid(pid_t &LmtdPid)
{
    /* The default return is failure, couldn't read the pid file. */
    bool Ok = false;
    
    /* If the lmtd.pid file is missing then we have a problem.  Unfortunately
     * I don't see what to do about it.
     *    One possible reason for no lmtd.pid file is that the lmtd process
     * hasn't finished starting yet.  To ensure this isn't the problem we
     * give it a while to appear. */
    struct stat StatBuf;
    if (TRY_WITH_TIMEOUT(stat(LMTD_PID_FILE, &StatBuf) == 0))
    {
        FILE * PidFile = fopen(LMTD_PID_FILE, "r");
        if (PidFile == NULL)
            perror("Error opening lmtd pid file");
        else
        {
            if (fscanf(PidFile, "%d", &LmtdPid) == 1)
                Ok = true;
            else
                perror("Malformed lmtd pid file");
            fclose(PidFile);
        }
    }
    else
        /* Damn: there's still no .pid file, even after waiting.  Can't do
         * anything more, unfortunately. */
        printf("Can't find %s\n", LMTD_PID_FILE);
    
    return Ok;
}


static bool ReadLmtdState(pid_t &LmtdPid, int &Offset)
{
    bool Ok = false;
    FILE * StateFile = fopen(LMTD_STATE_FILE, "r");
    if (StateFile != NULL)
    {
        if (fscanf(StateFile, "%d %d", &LmtdPid, &Offset) == 2)
            Ok = true;
        else
            perror("Malformed lmtd state file");
        fclose(StateFile);
    }
    return Ok;
}



/* Kills the LMTD if necessary, returning true iff a new LMTD needs to be
 * started. */

static bool KillLmtd(int detune)
{
    pid_t LmtdPid, SavedPid;
    int SavedOffset;
    bool ReadPid = ReadLmtdPid(LmtdPid);
    bool ReadSaved = ReadLmtdState(SavedPid, SavedOffset);
    /* Only keep the currently running LMTD if the state we've read is
     * consistent and if the current detune offset is the one we want. */
    bool KeepLmtd =
        ReadSaved  &&
        ReadPid  &&
        LmtdPid == SavedPid &&
        SavedOffset == detune;
    
    /* Only actually kill the LMTD if we need to and we have a valid pid to
     * kill. */
    if (!KeepLmtd && ReadPid)
    {
        /* Now kill the old lmtd and wait for it to finish. */
        if (TEST_(kill, LmtdPid, SIGTERM))
        {
            /* Wait for the process to finish.  I'd love to call waitpid(2),
             * but alas this only works for child processes, so instead we
             * keep probing for its presence until it's gone. */
            if (!TRY_WITH_TIMEOUT(kill(LmtdPid, 0) != 0))
                printf("Timed out waiting for LMTD to die\n");
        }
    }
    return ! KeepLmtd;
}


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

            /* Ensure the new process has a clean environment. */
            clearenv();

            /* Set a sensible home directory. */
            TEST_(chdir, "/");

            /* Restore default signal handling.  I'd like to hope this step
             * isn't needed. */
            for (int i = 0; i < NSIG; i ++)
                signal(i, SIG_DFL);

            /* Enable all signals. */
            sigset_t sigset;
            TEST_(sigfillset, &sigset);
            TEST_(sigprocmask, SIG_UNBLOCK, &sigset, NULL);

            /* Close all the open file handles.  The new lmtd gets into trouble
             * if we don't do this. */
            for (int i = 0; i < sysconf(_SC_OPEN_MAX); i ++)
                close(i);

            /* Finally we can actually exec the new process... */
            TEST_(execv, Process, argv);

            // Consider using execve(2) instead.
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


/* The phase advance per sample for the intermediate frequency generator is
 * controlled by writing to register 0x14014004.  The phase advance is in
 * units of 2^32*f_if/f_s, where f_if is the intermediate frequency and f_s
 * is the sample clock frequency. 
 *     If we write P=prescale, D=decimation, H=bunches per turn and F=detune
 * then the detuned sample frequency is
 *
 *          f_s = (D/H + F/HP) f_rf
 *
 * and the desired intermediate frequency scaling factor (to ensure that the
 * resampled RF frequency is equal to the IF) is
 *
 *          N = 2^32 frac(f_rf/f_s).
 */

#define IF_REGISTER     0x14014004

static void SetIntermediateFrequency(int detune)
{
    /* Access the IF register by mapping it into memory. */
    const int PageSize = getpagesize();
    const int PageMask = PageSize - 1;
    int DevMem = open("/dev/mem", O_RDWR | O_SYNC);
    char * Page = (char *) mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED,
        DevMem, IF_REGISTER & ~PageMask);
    int * Register = (int *) (Page + (IF_REGISTER & PageMask));

    /* Compute N = 2^32 frac(HP/(DP+F)) using floating point arithmetic. */
    double frf_fs =
        (BunchesPerTurn*LmtdPrescale)/
        ((double) SamplesPerTurn*LmtdPrescale + detune);
    int NyquistFactor = BunchesPerTurn / SamplesPerTurn;
    double two_32 = 65536.0*65536.0;    // (double)(1<<32) ?!  Alas not...
    int NewN = int(two_32 * (frf_fs - NyquistFactor));
    *Register = NewN;
        
    munmap(Page, PageSize);
    close(DevMem);
}



/* Switches the LMTD between two states, tuned or detuned, and also optionally
 * implements "double-detune".
 *
 * This implementation is a bit involved, as we need to kill and restart the
 * LMTD process each time the setting changes.
 *
 * This code will either disappear as the detune is built into libera, or
 * else will need rework to make it more portable. */

static void UpdateLmtdState()
{
    /* Work out how much we're going to detune by. */
    int detune = DetuneState == LMTD_TUNED ? 0 : DetuneFactor;

    /* See if we need a new LMTD.  We only restart the LMTD if we have to, as
     * doing so causes synchronisation to be lost. */
    if (KillLmtd(detune))
    {
        /* Now, whatever happened above, fire up a new lmtd with the command
         * line
         *      lmtd -p<prescale> -d<samples> -f<detune>
         */
        char _p[20];  sprintf(_p, "-p%d", LmtdPrescale);
        char _d[20];  sprintf(_d, "-d%d", SamplesPerTurn);
        char _f[20];  sprintf(_f, "-f%d", detune);
        char * Args[] = { "/opt/bin/lmtd", _p, _d, _f, NULL };
        DetachProcess(Args[0], Args);

        /* Try to read the new lmtd pid, and if successful write a new state
         * file.  Otherwise ensure we get rid of any state file. */
        pid_t LmtdPid;
        if (ReadLmtdPid(LmtdPid))
        {
            /* Record the current LMTD parameters. */
            FILE * LmtdState = fopen(LMTD_STATE_FILE, "w");
            if (LmtdState == NULL)
                perror("Unable to write LMTD state");
            else
            {
                fprintf(LmtdState, "%d %d\n", LmtdPid, detune);
                fclose(LmtdState);
            }
        }
        else
            unlink(LMTD_STATE_FILE);
    }

    /* Finally sort out the double detune.  We always apply this step, as
     * it has no side effects. */
    if (DetuneState == LMTD_DETUNED)
        /* In single detune state we don't apply any detune to the IF. */
        detune = 0;
    SetIntermediateFrequency(detune);
}



/*****************************************************************************/
/*                                                                           */
/*                           Clock Synchronisation                           */
/*                                                                           */
/*****************************************************************************/


class SYNCHRONISE_CLOCKS : public THREAD, I_EVENT
{
public:
    SYNCHRONISE_CLOCKS() :
        SyncState(false)
    {
        Dummy = false;
        RunningState = WAITING;
        
        PUBLISH_METHOD_OUT(bo, "CF:SYNC", SynchroniseClocks, Dummy);
        Publish_bi("CF:SYNCSTATE", SyncState);
        
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
                SetClockTime(NewTime);

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


    /* Called during shutdown to request orderly thread termination. */
    void OnTerminate()
    {
        Lock();
        RunningState = TERMINATING;
        Signal();
        Unlock();
    }


    /* This is called in response to processing the CF:SYNC record, to tell
     * us that the next trigger will be a clock synchronisation trigger.  We
     * wake up the main thread to do this work for us. */
    bool SynchroniseClocks(bool)
    {
        Lock();
        if (RunningState == WAITING)
            RunningState = SYNCHRONISING;
        SyncState.Write(true);
        Signal();
        Unlock();
        return true;
    }
        


    /* Wrappers for synchronisation. */

    void Lock()
    {
        TEST_(pthread_mutex_lock, &Mutex);
    }
    
    void Unlock()
    {
        TEST_(pthread_mutex_unlock, &Mutex);
    }

    void Signal()
    {
        TEST_(pthread_cond_signal, &Condition);
    }

    void Wait()
    {
        /* Note that Mutex must be locked before calling this. */
        TEST_(pthread_cond_wait, &Condition, &Mutex);
    }
    
    
    TRIGGER SyncState;
    bool Dummy;

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




/****************************************************************************/
/*                                                                          */

static bool Dummy;

static void DoReboot()
{
    char * Args[] = { "/sbin/halt", "/sbin/reboot", NULL };
    DetachProcess(Args[0], Args);
}


static void DoRestart()
{
    char * Args[] = { "/etc/init.d/epics", "restart", NULL };
    DetachProcess(Args[0], Args);
}



bool InitialiseTimestamps(
    int _LmtdPrescale, int _SamplesPerTurn, int _BunchesPerTurn)
{
    if (!InitialiseSynchroniseClocks())
        return false;
    
    LmtdPrescale   = _LmtdPrescale;
    SamplesPerTurn = _SamplesPerTurn;
    BunchesPerTurn = _BunchesPerTurn;
    
    PUBLISH_CONFIGURATION("CF:LMTD", mbbo, DetuneState, UpdateLmtdState);
    PUBLISH_CONFIGURATION("CF:DETUNE", longout, DetuneFactor, UpdateLmtdState);

    UpdateLmtdState();

    /* Also a couple of rather miscellanous PVs that could go anywhere.  They
     * currently live here so that they can share the DetachProcess() api. */
    PUBLISH_FUNCTION_OUT(bo, "REBOOT",  Dummy, DoReboot);
    PUBLISH_FUNCTION_OUT(bo, "RESTART", Dummy, DoRestart);

    return true;
}


void TerminateTimestamps()
{
    TerminateSynchroniseClocks();
}
