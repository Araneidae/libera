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


/* LMTD configuration: each of these is written to the LMTD. */
static int SampleClockDetune = 0;   // CK:DETUNE - frequency offset
static int IfClockDetune = 0;       // CK:IFOFF - IF offset ("double detune")
static int PhaseOffset = 0;         // CK:PHASE - phase offset

static bool UseSystemTime = false;



/* Sends a command to the LMTD daemon.  We close the file handle between
 * commands to allow other (generally debugging) commands to be sent from
 * other processes. */

static void SendLmtdCommand(const char * format, ...)
{
    FILE * LmtdCommandFile = fopen(LMTD_COMMAND_FIFO, "w");
    if (LmtdCommandFile == NULL)
        perror("Unable to open LMTD command fifo");
    else
    {
        va_list args;
        va_start(args, format);
        vfprintf(LmtdCommandFile, format, args);
        fprintf(LmtdCommandFile, "\n");
        fclose(LmtdCommandFile);
    }
}



/* Manage the configured LMTD state by sending the appropropriate commands to
 * the LMTD. */

static void UpdateLmtdState()
{
    SendLmtdCommand("o%d", SampleClockDetune);
    SendLmtdCommand("n%d", IfClockDetune + SampleClockDetune);
    SendLmtdCommand("p%d", PhaseOffset);
}



/* This thread manages the lmtd state reporting thread.  All status reported
 * from the LMTD is read and converted into updating EPICS PVs. */

class LMTD_MONITOR : public THREAD
{
public:
    LMTD_MONITOR() : THREAD("LMTD_MONITOR")
    {
        LmtdState = LMTD_NO_CLOCK;
        DacSetting = 0;
        PhaseError = 0;
        FrequencyError = 0;
        MachineClockSynchronised = SYNC_NO_SYNC;
        SystemClockSynchronised  = SYNC_NO_SYNC;
        LstdLocked = false;

        Publish_mbbi  ("CK:MC_LOCK", LmtdState);
        Publish_bi    ("CK:SC_LOCK", LstdLocked);
        Publish_longin("CK:DAC", DacSetting);
        Publish_longin("CK:PHASE_E", PhaseError);
        Publish_longin("CK:FREQ_E", FrequencyError);
        Publish_mbbi  ("CK:MC_SYNC", MachineClockSynchronised);
        Publish_mbbi  ("CK:SC_SYNC", SystemClockSynchronised);
        Interlock.Publish("CK");
    }


    /* These methods will migrate into SC status reports from the PLL daemon,
     * but for the moment we hear from the clock synchronisation thread
     * directly. */
    void ScWaiting()
    {
        if (LstdLocked)
            SystemClockSynchronised = SYNC_TRACKING;
    }

    void ScTriggered()
    {
        if (SystemClockSynchronised == SYNC_TRACKING)
            SystemClockSynchronised = SYNC_SYNCHRONISED;
    }


    bool IsSystemClockSynchronised()
    {
        return SystemClockSynchronised == SYNC_SYNCHRONISED;
    }
    

private:
    /* Decodes a single status line read from the LMTD and ensures that
     * updates are reported as appropriate. */
    void ProcessStatusLine(const char * Line)
    {
        if (sscanf(Line, "%d %d %d %d %d\n",
                &LmtdState, &FrequencyError, &PhaseError, &DacSetting,
                &MachineClockSynchronised) != 5)
            printf("Error scanning lmtd status line \"%s\"\n", Line);

        /* Pick up the LSTD state: this one we just pick up directly from
         * CSPI.  Longer term we probably want to integrate LSTD and LMTD
         * together and just use the status pipe. */
        bool Dummy;
        GetClockState(Dummy, LstdLocked);
        /* Also update the synchronisation state accordingly: on loss of lock
         * force the state to no sync. */
        if (!LstdLocked)
            SystemClockSynchronised = SYNC_NO_SYNC;
    }
    
    void Thread()
    {
        FILE * LmtdStatus = fopen(LMTD_STATUS_FIFO, "r");
        if (LmtdStatus == NULL)
        {
            perror("Unable to open lmtd status pipe");
            return;
        }
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
    int MachineClockSynchronised;
    int SystemClockSynchronised;
    bool LstdLocked;

    INTERLOCK Interlock;
};



/* Publish a ticking record to publish the fact that trigger has been
 * processed, together with timing information for this trigger. */

class TICK_TRIGGER : I_EVENT
{
public:
    TICK_TRIGGER()
    {
        NtpTimeString[0] = '\0';
        SystemTimeString[0] = '\0';
        
        /* Publishing the interlock will also make MCL and MCH fields
         * available with machine clock information. */
        Interlock.Publish("CK", true, "TIME", "TIME_DONE");
        Publish_stringin("CK:TIME_NTP", NtpTimeString);
        Publish_stringin("CK:TIME_SC", SystemTimeString);
        
        RegisterTriggerEvent(*this, PRIORITY_TICK);
    }
private:

    void FormatTimeString(struct timespec st, EPICS_STRING &String)
    {
        /* Start by converting ns into microseconds: the nanosecond detail is
         * not really meaningful or useful. */
        int usec = (st.tv_nsec + 500) / 1000;
        if (usec >= 1000000)
        {
            usec -= 1000000;
            st.tv_sec += 1;
        }
        /* Convert the timestamp into a sensible string. */
        struct tm Tm;
        gmtime_r(&st.tv_sec, &Tm);
        snprintf(String, sizeof(EPICS_STRING),
            "%04d/%02d/%02d %02d:%02d:%02d.%06d",
            1900 + Tm.tm_year, Tm.tm_mon + 1, Tm.tm_mday,
            Tm.tm_hour, Tm.tm_min, Tm.tm_sec, usec);
    }
    
    void OnEvent(int)
    {
        Interlock.Wait();
        
        /* The only way to get a timestamp from this trigger is to read some
         * triggered data.  Read the least possible amount right now! */
        CSPI_TIMESTAMP Timestamp;
        LIBERA_ROW OneRow;
        ReadWaveform(1, 1, &OneRow, Timestamp);

        /* Format the two versions of the time into the approriate fields. */
        FormatTimeString(Timestamp.st, SystemTimeString);
        struct timespec NtpTime;
        TEST_(clock_gettime, CLOCK_REALTIME, &NtpTime);
        FormatTimeString(NtpTime, NtpTimeString);

        Interlock.Ready(Timestamp);
    }

    INTERLOCK Interlock;
    EPICS_STRING NtpTimeString;
    EPICS_STRING SystemTimeString;
};



static LMTD_MONITOR * LmtdMonitorThread = NULL;



/*****************************************************************************/
/*                                                                           */
/*                           Clock Synchronisation                           */
/*                                                                           */
/*****************************************************************************/


/* This class manages system clock synchronisation.  This involves bringing
 * our internal time (as managed by ntp) in step with an external trigger.
 * The external trigger should occur on the second, so we need to repeatedly
 * re-arm the SC trigger with the next anticipated second.  Thus we need to
 * do this in a separate thread. */

class SYNCHRONISE_CLOCKS : public LOCKED_THREAD, I_EVENT
{
public:
    SYNCHRONISE_CLOCKS() : LOCKED_THREAD("SYNCHRONISE_CLOCKS")
    {
        SystemClockSynchronising = false;
        MachineClockSynchronising = false;
        
        PUBLISH_METHOD_ACTION("CK:SC_SYNC", SynchroniseSystemClock);
        PUBLISH_METHOD_ACTION("CK:MC_SYNC", SynchroniseMachineClock);
        
        RegisterTriggerSetEvent(*this, PRIORITY_SYNC);
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
            
            while (SystemClockSynchronising)
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


    /* This is called in response to processing the CK:SYNCSC record, to tell
     * us that the next trigger will be a system clock synchronisation
     * trigger.  We wake up the main thread to do this work for us. */
    bool SynchroniseSystemClock()
    {
        Lock();
        SystemClockSynchronising = true;
        LmtdMonitorThread->ScWaiting();
        Signal();
        Unlock();
        return true;
    }


    /* This is called in response to processing the CK:SYNCMC record: the
     * next trigger is a machine clock synchronisation trigger.  In response
     * we need to let LMTD know that a sync is about to happen.
     *    Because we need to receive the trigger (which is shared with SC
     * synchronisation) we need to be part of this thread. */
    bool SynchroniseMachineClock()
    {
        Lock();
        MachineClockSynchronising = true;
        SendLmtdCommand("s%d", SYNC_TRACKING);
        SetMachineClockTime();
        Unlock();
        return true;
    }


    /* This is called when the TRIGSET event is received: this informs us
     * that the clock setting trigger has been received and so clock setting
     * is complete. */
    void OnEvent(int)
    {
        Lock();
        if (MachineClockSynchronising)
        {
            MachineClockSynchronising = false;
            SendLmtdCommand("s%d", SYNC_SYNCHRONISED);
        }
        if (SystemClockSynchronising)
        {
            SystemClockSynchronising = false;
            LmtdMonitorThread->ScTriggered();
        }
        Signal();
        Unlock();
    }
    

    bool MachineClockSynchronising;
    bool SystemClockSynchronising;
};



static SYNCHRONISE_CLOCKS * SynchroniseThread = NULL;



bool InitialiseTimestamps()
{
    PUBLISH_CONFIGURATION(longout, "CK:DETUNE", 
        SampleClockDetune, UpdateLmtdState);
    PUBLISH_CONFIGURATION(longout, "CK:IFOFF", 
        IfClockDetune, UpdateLmtdState);
    PUBLISH_CONFIGURATION(longout, "CK:PHASE", 
        PhaseOffset, UpdateLmtdState);
    PUBLISH_CONFIGURATION(bo, "CK:TIMESTAMP", UseSystemTime, NULL_ACTION);
    
    new TICK_TRIGGER();
    
    SynchroniseThread = new SYNCHRONISE_CLOCKS;
    if (!SynchroniseThread->StartThread())
        return false;
    
    LmtdMonitorThread = new LMTD_MONITOR;
    if (!LmtdMonitorThread->StartThread())
        return false;

    /* Program the LMTD to the required settings. */
    UpdateLmtdState();

    return true;
}


void TerminateTimestamps()
{
    if (SynchroniseThread != NULL)
        SynchroniseThread->Terminate();
    if (LmtdMonitorThread != NULL)
        LmtdMonitorThread->Terminate();
}


bool UseLiberaTimestamps()
{
    return UseSystemTime && LmtdMonitorThread->IsSystemClockSynchronised();
}
