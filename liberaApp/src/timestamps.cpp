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
#include <poll.h>
#include <ctype.h>

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
/*                        Clock PLL Daemon Interface                         */
/*                                                                           */
/*****************************************************************************/


/* PLL configuration: each of these is written to the PLL daemon. */
static int SampleClockDetune = 0;   // CK:DETUNE - frequency offset
static int IfClockDetune = 0;       // CK:IFOFF - IF offset ("double detune")
static int PhaseOffset = 0;         // CK:PHASE - phase offset

static bool UseSystemTime = false;
static bool Verbose = false;
static bool EnableOpenLoop = false;



/* Sends a command to the clockPll daemon.  We close the file handle between
 * commands to allow other (generally debugging) commands to be sent from
 * other processes. */

static void SendPllCommand(const char * format, ...)
{
    FILE * PllCommandFile = fopen(CLOCK_PLL_COMMAND_FIFO, "w");
    if (PllCommandFile == NULL)
        perror("Unable to open clockPll command fifo");
    else
    {
        va_list args;
        va_start(args, format);
        vfprintf(PllCommandFile, format, args);
        fprintf(PllCommandFile, "\n");
        fclose(PllCommandFile);
    }
}



/* Brings the entire state of the clock PLL daemon up to date.  It's safe to
 * call this repeatedly. */

static void UpdatePllState()
{
    SendPllCommand("mo%d", SampleClockDetune);
    SendPllCommand("mp%d", PhaseOffset);
    SendPllCommand("n%d",  IfClockDetune + SampleClockDetune);
    
    SendPllCommand("mv%d", Verbose);
    SendPllCommand("sv%d", Verbose);
    SendPllCommand("mc%d", EnableOpenLoop);
    SendPllCommand("sc%d", EnableOpenLoop);
}



/* Class for reading lines with timeout. */

class GETLINE
{
public:
    GETLINE(const char * FileName, int BufferLength) :
        FileName(FileName),
        BufferLength(BufferLength),
        Buffer(new char[BufferLength])
    {
        FileFd.fd = -1;
        FileFd.events = POLLIN;
        InPointer = 0;
    }

    ~GETLINE() { Close(); }

    bool Ok() { return FileFd.fd != -1; }

    bool Open(int Timeout)
    {
        FileFd.fd = open(FileName, O_RDONLY | O_NONBLOCK);
        /* A little hack: if we can't open the file then sleep a little.
         * This will give the rest of the system a bit more time to do
         * something about it. */
        if (FileFd.fd == -1)
            sleep(Timeout / 1000);
        return FileFd.fd != -1;
    }

    void Close()
    {
        if (Ok())
        {
            close(FileFd.fd);
            FileFd.fd = -1;
        }
    }

    bool ReadLine(char * Line, int LineLength, int Timeout)
    {
        /* Try to open the file. */
        if (!Ok()  &&  !Open(Timeout))
            return false;
        
        /* Read from the pipe until either there's a line in the buffer or
         * we time out. */
        char * newline;
        while (
            newline = (char *) memchr(Buffer, '\n', InPointer),
            newline == NULL)
        {
            /* Check for possible buffer overflow.  If the buffer has filled
             * up without a newline appearing then the simplest thing we can
             * do is throw it all away and start again. */
            if (!TEST_OK(InPointer < BufferLength))
                InPointer = 0;

            int Read;
            bool Ok =
                /* Wait for input to arrive with specified timeout in
                 * milliseconds. */
                TEST_IO(Read, poll, &FileFd, 1, Timeout)  &&
                /* Check that the file is readable; if not, timed out. */
                Read == 1  &&
                /* Try to read the incoming data. */
                TEST_IO(Read,
                    read, FileFd.fd,
                    Buffer + InPointer, BufferLength - InPointer);
            if (!Ok)
                return false;
            else if (Read == 0)
            {
                Close();
                return false;
            }
            else
                InPointer += Read;
        }
        
        /* Copy one line of the read result back to the caller. */
        *newline++ = '\0';
        strncpy(Line, Buffer, LineLength);
        /* If there is anything left in the buffer then for simplicity simply
         * move it up to the start.  This is easier than managing a circular
         * buffer, and is a rare case anyway. */
        int Residue = Buffer + InPointer - newline;
        memmove(Buffer, newline, Residue);
        InPointer = Residue;
        return true;
    }
    
private:
    const char * FileName;      // Name of file to open
    const int BufferLength;     // Length of allocated buffer
    char * Buffer;              // Line buffer
    struct pollfd FileFd;       // File handle and poll() event mask
    int InPointer;              // Length of read data in buffer
};



/* Handles the processing of the interface for one of two clocks. */

class CLOCK_MONITOR
{
public:
    CLOCK_MONITOR(const char * Clock) :
        PrefixId(tolower(Clock[0]))
    {
        State = 0;
        Synchronised = false;
        DacSetting = 0;
        PhaseError = 0;
        FrequencyError = 0;
        
        char Prefix[20];
        sprintf(Prefix, "CK:%s_", Clock);
        Publish_mbbi  (Concat(Prefix, "LOCK"),    State);
        Publish_mbbi  (Concat(Prefix, "SYNC"),    Synchronised);

        Publish_longin(Concat(Prefix, "DAC"),     DacSetting);
        Publish_longin(Concat(Prefix, "PHASE_E"), PhaseError);
        Publish_longin(Concat(Prefix, "FREQ_E"),  FrequencyError);

        PUBLISH_METHOD_OUT(longout, Concat(Prefix, "DAC"),
            SetDac, DacSetting);

        StatusInterlock.Publish("CK", false,
            Concat(Clock, "_S_TRIG"), Concat(Clock, "_S_DONE"));
        VerboseInterlock.Publish("CK", false,
            Concat(Clock, "_V_TRIG"), Concat(Clock, "_V_DONE"));
    }

    void ProcessStatusLine(const char *Line)
    {
        switch(*Line++)
        {
            case 's':
                StatusInterlock.Wait();
                TEST_OK(sscanf(Line, "%d %d", &State, &Synchronised) == 2);
                StatusInterlock.Ready();
                break;
            case 'v':
                VerboseInterlock.Wait();
                TEST_OK(sscanf(Line, "%d %d %d",
                    &FrequencyError, &PhaseError, &DacSetting) == 3);
                VerboseInterlock.Ready();
                break;
            default:
                TEST_OK(false);
        }
    }

    void ProcessStatusError()
    {
        StatusInterlock.Wait();
        State = 0;
        Synchronised = false;
        StatusInterlock.Ready();
        
        VerboseInterlock.Wait();
        DacSetting = 0;
        PhaseError = 0;
        FrequencyError = 0;
        VerboseInterlock.Ready();
    }

    bool IsSynchronised()
    {
        return Synchronised == SYNC_SYNCHRONISED;
    }
    
private:
    CLOCK_MONITOR();
    
    bool SetDac(int NewDac)
    {
        if (EnableOpenLoop)
            SendPllCommand("%cd%d", PrefixId, NewDac);
        return EnableOpenLoop;
    }
    

    const char PrefixId;
        
    int State;
    int Synchronised;
    int DacSetting;
    int PhaseError;
    int FrequencyError;

    INTERLOCK StatusInterlock;
    INTERLOCK VerboseInterlock;
};


/* This thread manages the PLL state reporting thread.  All status reported
 * from the clockPll daemon is read and converted into updating EPICS PVs. */

class CLOCK_PLL_MONITOR : public THREAD
{
public:
    CLOCK_PLL_MONITOR() : THREAD("CLOCK_PLL_MONITOR")
    {
        MC_monitor = new CLOCK_MONITOR("MC");
        SC_monitor = new CLOCK_MONITOR("SC");
    }

    bool IsSystemClockSynchronised()
    {
        return SC_monitor->IsSynchronised();
    }
    

private:
    /* Decodes a single status line read from clockPll and ensures that
     * updates are reported as appropriate. */
    void ProcessStatusLine(const char * Line)
    {
        switch (*Line++)
        {
            case 'm':
                MC_monitor->ProcessStatusLine(Line);
                break;
            case 's':
                SC_monitor->ProcessStatusLine(Line);
                break;
            case 'x':
                /* On receipt of reset reinitialise the PLL daemon. */
                UpdatePllState();
                break;
            default:
                printf("Invalid PLL status line: \"%s\"\n", Line);
                ProcessStatusError();
                break;
        }
    }

    /* If we lose communication with the PLL daemon then switch all our state
     * into the default error state. */
    void ProcessStatusError()
    {
        MC_monitor->ProcessStatusError();
        SC_monitor->ProcessStatusError();
    }
    
    void Thread()
    {
        StartupOk();
        
        GETLINE PllStatus(CLOCK_PLL_STATUS_FIFO, 128);
        while (Running())
        {
            char Line[128];
            if (PllStatus.ReadLine(Line, sizeof(Line), 2000))
                ProcessStatusLine(Line);
            else
                ProcessStatusError();
        }
    }

    CLOCK_MONITOR * MC_monitor;
    CLOCK_MONITOR * SC_monitor;
};


static CLOCK_PLL_MONITOR * PllMonitorThread = NULL;



/* Publish a ticking record to publish the fact that trigger has been
 * processed, together with timing information for this trigger. */

class TICK_TRIGGER : I_EVENT
{
public:
    TICK_TRIGGER()
    {
        NtpTimeString[0] = '\0';
        SystemTimeString[0] = '\0';
        MissedEventCount = 0;
        
        /* Publishing the interlock will also make MCL and MCH fields
         * available with machine clock information. */
        Interlock.Publish("CK", true, "TIME", "TIME_DONE");
        Publish_stringin("CK:TIME_NTP", NtpTimeString);
        Publish_stringin("CK:TIME_SC", SystemTimeString);
        Publish_longin("CK:MISSED", MissedEventCount);
        
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
            "%04d-%02d-%02d %02d:%02d:%02d.%06d",
            1900 + Tm.tm_year, Tm.tm_mon + 1, Tm.tm_mday,
            Tm.tm_hour, Tm.tm_min, Tm.tm_sec, usec);
    }
    
    void OnEvent(int MissedEvents)
    {
        Interlock.Wait();
        
        /* The only way to get a timestamp from this trigger is to read some
         * triggered data.  Read the least possible amount right now! */
        LIBERA_TIMESTAMP Timestamp;
        LIBERA_ROW OneRow;
        ReadWaveform(1, 1, &OneRow, Timestamp);

        /* Format the two versions of the time into the approriate fields. */
        FormatTimeString(Timestamp.st, SystemTimeString);
        struct timespec NtpTime;
        TEST_(clock_gettime, CLOCK_REALTIME, &NtpTime);
        FormatTimeString(NtpTime, NtpTimeString);

        MissedEventCount = MissedEvents;

        /* Fix up the timestamp if necessary before publishing so that we use
         * the same timestamps as everybody else.  This is the same test as
         * in AdjustTimestamp(), but here we use the NtpTime we've already
         * fetched to avoid confusion. */
        if (!(UseSystemTime && PllMonitorThread->IsSystemClockSynchronised()))
            memcpy(&Timestamp.st, &NtpTime, sizeof(Timestamp.st));
        Interlock.Ready(Timestamp);
    }

    INTERLOCK Interlock;
    EPICS_STRING NtpTimeString;
    EPICS_STRING SystemTimeString;
    int MissedEventCount;
};




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
        SendPllCommand("ss%d", SYNC_TRACKING);
        Signal();
        Unlock();
        return true;
    }


    /* This is called in response to processing the CK:SYNCMC record: the
     * next trigger is a machine clock synchronisation trigger.  In response
     * we need to let clockPll know that a sync is about to happen.
     *    Because we need to receive the trigger (which is shared with SC
     * synchronisation) we need to be part of this thread. */
    bool SynchroniseMachineClock()
    {
        Lock();
        MachineClockSynchronising = true;
        SendPllCommand("ms%d", SYNC_TRACKING);
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
            SendPllCommand("ms%d", SYNC_SYNCHRONISED);
        }
        if (SystemClockSynchronising)
        {
            SystemClockSynchronising = false;
            SendPllCommand("ss%d", SYNC_SYNCHRONISED);
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
        SampleClockDetune, UpdatePllState);
    PUBLISH_CONFIGURATION(longout, "CK:IFOFF", 
        IfClockDetune, UpdatePllState);
    PUBLISH_CONFIGURATION(longout, "CK:PHASE", 
        PhaseOffset, UpdatePllState);
    PUBLISH_CONFIGURATION(bo, "CK:TIMESTAMP", UseSystemTime, NULL_ACTION);
    PUBLISH_FUNCTION_OUT(bo, "CK:VERBOSE", Verbose, UpdatePllState);

    /* Open loop direct DAC control. */
    PUBLISH_FUNCTION_OUT(bo, "CK:OPEN_LOOP",  EnableOpenLoop, UpdatePllState);
    
    new TICK_TRIGGER();
    
    SynchroniseThread = new SYNCHRONISE_CLOCKS;
    if (!SynchroniseThread->StartThread())
        return false;
    
    PllMonitorThread = new CLOCK_PLL_MONITOR;
    if (!PllMonitorThread->StartThread())
        return false;

    /* Program the PLL daemon to the required settings. */
    UpdatePllState();

    return true;
}


void TerminateTimestamps()
{
    if (SynchroniseThread != NULL)
        SynchroniseThread->Terminate();
    if (PllMonitorThread != NULL)
        PllMonitorThread->Terminate();
}


void AdjustTimestamp(LIBERA_TIMESTAMP &Timestamp)
{
    /* Unless both the use of system time is enabled *and* the system clock
     * is currently synchronised, use current NTP time instead of the
     * reported Libera system clock. */
    if (!(UseSystemTime && PllMonitorThread->IsSystemClockSynchronised()))
        TEST_(clock_gettime, CLOCK_REALTIME, & Timestamp.st);
}
