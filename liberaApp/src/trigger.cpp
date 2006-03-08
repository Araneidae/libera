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

#include <initHooks.h>

#include "drivers.h"
#include "publish.h"
#include "hardware.h"
#include "trigger.h"



/*****************************************************************************/
/*                                                                           */
/*                              TRIGGER class                                */
/*                                                                           */
/*****************************************************************************/


TRIGGER::TRIGGER(bool InitialValue)
{
    Value = InitialValue;
    iIntr = NULL;
}

/* This method signals that the trigger is ready. */
bool TRIGGER::Ready()
{
    if (iIntr == NULL)
        return false;
    else
        return iIntr->IoIntr();
}

/* Epics support routine: actually, nothing to do! */
bool TRIGGER::read(bool &ValueRead)
{
    ValueRead = Value;
    return true;
}

/* Support I/O Intr functionality: overrides I_bo method. */
bool TRIGGER::EnableIoIntr(I_INTR & Intr)
{
    iIntr = &Intr;
    return true;
}

void TRIGGER::Write(bool NewValue)
{
    Value = NewValue;
    Ready();
}



/*****************************************************************************/
/*                                                                           */
/*                             INTERLOCK class                               */
/*                                                                           */
/*****************************************************************************/


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


INTERLOCK::INTERLOCK()
{
    /* If Wait() will be called before Ready() then start the semaphore with
     * an initial resource so we don't block immediately! */
    TEST_(sem_init, &Interlock, 0, 0);

    /* Add ourself to the list of interlocks so that we can all be signalled
     * once EPICS has finished initialisation.  All of our interlocks are
     * created during the main initialisation thread, so this does not need to
     * be thread safe. */
    Next = InterlockList;
    InterlockList = this;
    Value = true;
}


void INTERLOCK::Publish(const char * Prefix)
{
    Publish_bi(Concat(Prefix, ":TRIG"), Trigger);
    PUBLISH_METHOD_OUT(bo, Concat(Prefix, ":DONE"), ReportDone, Value);
}

void INTERLOCK::Ready()
{
    /* Inform EPICS, which is now responsible for calling ReportDone(). */
    Trigger.Ready();
}

void INTERLOCK::Wait()
{
    sem_wait(&Interlock);
}


/* This will be called when process is done.  Release the interlock.
 * This is normally called when DONE is processed, which should only be after
 * a chain of processing initiated by processing TRIG has completed.  It will
 * also be called when EPICS has finished initialisation. */

bool INTERLOCK::ReportDone(bool Done)
{
    TEST_(sem_post, &Interlock);
    return true;
}



/* Called when EPICS has finished initialisation: release all the waiting
 * interlocks so that normal processing can begin. */
  
void INTERLOCK::EpicsReady()
{
    for (INTERLOCK * Entry = InterlockList;
         Entry != NULL; Entry = Entry->Next)
        Entry->ReportDone(true);
}


INTERLOCK * INTERLOCK::InterlockList = NULL;


void InterlockInitHook(initHookState State)
{
    if (State == initHookAtEnd)
        INTERLOCK::EpicsReady();
}

bool InitialiseTriggers()
{
    return initHookRegister(InterlockInitHook) == 0;
}
