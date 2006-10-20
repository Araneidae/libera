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


/* Simple EPICS I/O Intr implementation through a bi record.  The act of
 * writing a boolean value to the trigger invokes EPICS processing for
 * interested records.
 *
 * This is used as a basic building block for passing Libera events up to the
 * interested EPICS records. */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <initHooks.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "events.h"
#include "trigger.h"



/*****************************************************************************/
/*                                                                           */
/*                              TRIGGER class                                */
/*                                                                           */
/*****************************************************************************/


TRIGGER::TRIGGER(bool InitialValue)
{
    Value = InitialValue;
    memset(&Timestamp, 0, sizeof(Timestamp));
    iIntr = NULL;
}

/* This method signals that the trigger is ready. */
bool TRIGGER::Ready(const struct timespec *NewTimestamp)
{
    /* If we've been given a timestamp then use that, otherwise fetch the
     * current time as our timestamp. */
    if (NewTimestamp == NULL)
        TEST_(clock_gettime, CLOCK_REALTIME, & Timestamp);
    else
        Timestamp = *NewTimestamp;

    /* If epics is listening, let it know. */
    if (iIntr == NULL)
        return false;
    else
        return iIntr->IoIntr();
}


/* Epics support routine: just return our value. */
bool TRIGGER::read(bool &ValueRead)
{
    ValueRead = Value;
    return true;
}

/* Support I/O Intr functionality. */
bool TRIGGER::EnableIoIntr(I_INTR & Intr)
{
    iIntr = &Intr;
    return true;
}

bool TRIGGER::GetTimestamp(struct timespec & Result)
{
    Result = Timestamp;
    return true;
}

void TRIGGER::Write(bool NewValue)
{
    Value = NewValue;
    Ready();
}



/*****************************************************************************/
/*                                                                           */
/*                              ENABLE class                                 */
/*                                                                           */
/*****************************************************************************/


ENABLE::ENABLE() :
    Persistent(Value)
{
    Value = true;
}

void ENABLE::Publish(const char * Prefix)
{
    const char * Name = Concat(Prefix, ":ENABLE");
    Publish_bo(Name, *this);
    Persistent.Initialise(Name);
}

bool ENABLE::init(bool &Result)
{
    Result = Value;
    return true;
}

bool ENABLE::write(bool NewValue)
{
    Value = NewValue;
    return true;
}

    

/*****************************************************************************/
/*                                                                           */
/*                             INTERLOCK class                               */
/*                                                                           */
/*****************************************************************************/


/* Supporting class for INTERLOCK class: this is used to block until EPICS
 * initialisation has completed.
 *     This code is annoyingly complicated to perform a rather simple task.
 * It might be worth abstracting the heart of this into the thread
 * component. */

class EPICS_READY
{
public:
    static bool Initialise()
    {
        /* Initialise our internal resources. */
        EpicsReady = false;
        TEST_(pthread_cond_init, &ReadyCondition, NULL);
        pthread_mutex_init(&ReadyMutex, NULL);
        /* Register with the EPICS initialisation process so that we will be
         * informed when initialisation is complete. */
        return initHookRegister(EpicsReadyInitHook) == 0;
    }

    /* Waits for EPICS initialisation to complete, or returns immediately if
     * this has already happened. */
    static void Wait()
    {
        /* We can get away without locking if the flag is set.  However, we
         * still have to test again after acquiring the lock, of course. */
        if (!EpicsReady)
        {
            Lock();
            while (!EpicsReady)
                TEST_(pthread_cond_wait, &ReadyCondition, &ReadyMutex);
            UnLock();
        }
    }
    
private:
    /* This hook routine is called repeatedly through the EPICS
     * initialisation process: however, we're only interested in the very
     * last report, telling us that initialisation is complete.  We then
     * communicate this to all interested waiting parties. */
    static void EpicsReadyInitHook(initHookState State)
    {
        if (State == initHookAtEnd)
        {
            /* Initialisation is finished.  We can now tell all listening
             * threads to go ahead. */
            Lock();
            EpicsReady = true;
            TEST_(pthread_cond_broadcast, &ReadyCondition);
            UnLock();
        }
    }

    static void Lock()
    {
        TEST_(pthread_mutex_lock, &ReadyMutex);
    }

    static void UnLock()
    {
        TEST_(pthread_mutex_unlock, &ReadyMutex);
    }

    static bool EpicsReady;
    static pthread_cond_t ReadyCondition;
    static pthread_mutex_t ReadyMutex;
};

bool EPICS_READY::EpicsReady = false;
pthread_cond_t EPICS_READY::ReadyCondition;
pthread_mutex_t EPICS_READY::ReadyMutex;



/* The choice of interlock mechanism is a little delicate, and depends
 * somewhat on what kinds of error we anticipate.  The most obvious interlock
 * is a semaphore, which we use.  However, a possible problem will be if more
 * than one DONE event occurs...  This is quite possible if the database is
 * tampered with directly.
 *
 * Another tiresome problem is that the interlocks will start operating
 * before EPICS has finished initialising.  Unfortunately we can't safely
 * call Ready() until EPICS has finished, as otherwise the event is quite
 * likely to be lost.
 *    To handle this we receive the EPICS finished event and use this to
 * signal all of the interlocks.  The implementation here relies on all of
 * the interlocks being created before EPICS is initialised, which is true. */


INTERLOCK::INTERLOCK() :
    /* As Wait() will be called before Ready() we start the semaphore with
     * an initial resource to avoid blocking immediately! */
    Interlock(true)
{
    Name = NULL;
    Value = true;
    MachineClockLow = 0;
    MachineClockHigh = 0;
}


void INTERLOCK::Publish(const char * Prefix, bool PublishMC)
{
    Name = Prefix;
    Publish_bi(Concat(Prefix, ":TRIG"), Trigger);
    PUBLISH_METHOD_OUT(bo, Concat(Prefix, ":DONE"), ReportDone, Value);

    if (PublishMC)
    {
        Publish_longin(Concat(Prefix, ":MCL"), MachineClockLow);
        Publish_longin(Concat(Prefix, ":MCH"), MachineClockHigh);
    }
}

void INTERLOCK::Ready(const CSPI_TIMESTAMP &Timestamp)
{
    if (&Timestamp == NULL)
        /* No timestamp: let the trigger use current time. */
        Trigger.Ready();
    else
    {
        /* Give the trigger the true timestamp, and update our internal
         * machine clock. */ 
        Trigger.Ready(&Timestamp.st);
        /* Return the machine clock in two pieces.  Because EPICS longs are
         * signed, we truncate both parts to 31 bits each. */
        MachineClockLow  = (int) (Timestamp.mt & 0x7FFFFFFF);
        MachineClockHigh = (int) ((Timestamp.mt >> 31) & 0x7FFFFFFF);
    }
}

void INTERLOCK::Wait()
{
    /* Before going on and waiting for the interlock we start by waiting for
     * EPICS to report that it has finished initialisation. */
    EPICS_READY::Wait();
    /* Now wait for EPICS to finish processing the previous update.
     * Unfortunately experience tells us that the post we're waiting for can
     * go astray.  To guard against this possibility we wait on a timeout and
     * go ahead anyway(!) if the event never arrives.  Of course, if events
     * have become permanently lost then we're dead...
     *    Oddly enough, this message does occasionally appear in the ioc log.
     * No idea why, as yet. */
    if (!Interlock.Wait(2))
        printf("%s:DONE timed out waiting for EPICS handshake\n", Name);
}


/* This will be called when process is done.  Release the interlock.
 * This is normally called when DONE is processed, which should only be after
 * a chain of processing initiated by processing TRIG has completed.  It will
 * also be called when EPICS has finished initialisation. */

bool INTERLOCK::ReportDone(bool Done)
{
    /* If the interlock was already ready when we signal it then something
     * has gone wrong. */
    if (Interlock.Signal())
        printf("%s:DONE unexpected extra signal\n", Name);
    return true;
}



/*****************************************************************************/
/*                                                                           */
/*                    Initialisation and Persistent State                    */
/*                                                                           */
/*****************************************************************************/

class TICK_TRIGGER : TRIGGER, I_EVENT
{
public:
    TICK_TRIGGER()
    {
        Publish_bi("TICK_TRIG", *this);
        RegisterTriggerEvent(*this, PRIORITY_TICK);
    }
private:
    void OnEvent()
    {
        Ready();
    }
};



bool InitialiseTriggers()
{
    /* Publish a ticking record to publish the fact that trigger has been
     * processed. */
    new TICK_TRIGGER();
    
    /* Ensure the trigger interlock mechanism (which needs to interact
     * carefully with EPICS) has been correctly initialise. */
    return EPICS_READY::Initialise();
}
