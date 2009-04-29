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


/* This code is concerned with receiving events from Libera and dispatching
 * them to interested parties. */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <signal.h>
#include <sys/select.h>
#include <pthread.h>
#include <assert.h>
#include <stdint.h>

#include "hardware.h"
#include "thread.h"

#include "events.h"



    
/*****************************************************************************/
/*                                                                           */
/*                         Event Processing Thread                           */
/*                                                                           */
/*****************************************************************************/


/* Special case processing for merging event parameters. */

int MergeParameters(
    int EventId, bool MergeRequired, int OldParameter, int NewParameter)
{
    switch (EventId)
    {
        case LIBERA_EVENT_INTERLOCK:
            /* For the interlock we keep the original parameter and throw new
             * values away: we're only interested in the *first* interlock
             * reason. */
            if (MergeRequired)
                return OldParameter;
            else
                return NewParameter;
        case LIBERA_EVENT_TRIGGET:
        case LIBERA_EVENT_PM:
            /* For normal and postmortem triggers just count the number of
             * missed triggers. */
            if (MergeRequired)
                return OldParameter + 1;
            else
                return 0;
        case LIBERA_EVENT_TRIGSET:
            /* For synchronisation triggers complain if we miss a trigger:
             * this shouldn't happen. */
            if (MergeRequired)
                printf("TRIGSET trigger missed!\n");
            return 0;
        default:
            /* Safe default action is to discard the old value. */
            return NewParameter;
    }
}



class EVENT_DISPATCHER : public LOCKED_THREAD
{
public:
    EVENT_DISPATCHER() :
        LOCKED_THREAD("EVENT_DISPATCHER"),
        signal(false)
    {
        /* Initialise the handler and event tables to empty. */
        for (int i = 0; i < EVENT_TABLE_SIZE; i ++)
            EventTable[i].Valid = false;
        for (int i = 0; i < HANDLER_TABLE_SIZE; i ++)
            HandlerTable[i].Handler = NULL;
    }


    /* Adds the given event to the list of events supported by this
     * dispatcher. */
    void EnableEvent(int EventId)
    {
        /* Find a free table entry (and check that the event is not already
         * defined). */
        for (int i = 0; i < EVENT_TABLE_SIZE; i ++)
            if (!EventTable[i].Valid)
            {
                EventTable[i].Valid = true;
                EventTable[i].EventId = EventId;
                EventTable[i].Occurred = false;
                return;
            }
            else
                assert(EventTable[i].EventId != EventId);
        assert(false);  // Oops: the table is full!
    }


    /* Register a handler for a particular event type. */
    void Register(I_EVENT & EventHandler, int EventId, int Index)
    {
        assert(
            0 <= Index  &&  Index < HANDLER_TABLE_SIZE  &&
            HandlerTable[Index].Handler == NULL);

        /* Record the handler for this event. */
        HandlerTable[Index].EventId = EventId;
        HandlerTable[Index].Handler = &EventHandler;
    }


    /* Event mask derived from the set of enabled events. */
    int EventMask()
    {
        int Mask = 0;
        for (int i = 0; i < EVENT_TABLE_SIZE; i ++)
            if (EventTable[i].Valid)
                Mask |= EventTable[i].EventId;
        return Mask;
    }


    /* This is called outside of the event receiver thread to notify the
     * thread that there is an event to process. */
    void NotifyEvent(int EventId, int EventParameter)
    {
        /* Find the associated table entry. */
        for (int i = 0; i < EVENT_TABLE_SIZE; i ++)
        {
            EVENT_TABLE &Event = EventTable[i];
            if (Event.Valid  &&  Event.EventId == EventId)
            {
                /* Process the event.  Preliminary processing here (in
                 * EVENT_RECEIVER context), and the rest of the processing
                 * will occur when the event is now dispatched. */
                THREAD_LOCK(this);
                Event.Parameter = MergeParameters(
                    EventId, Event.Occurred, Event.Parameter, EventParameter);
                Event.Occurred = true;
                THREAD_UNLOCK();
                signal.Signal();
                return;
            }
        }

        /* If we fall through to here the event was never handled.  This
         * really shouldn't happen: we shouldn't receive the event if we
         * didn't register an interest in it! */
        printf("Unhandled event %d (%d) ignored\n", EventId, EventParameter);
    }

    
    
private:
    void Thread()
    {
        StartupOk();

        while (Running())
        {
            /* Wait for something to happen. */
            signal.Wait();
            
            /* Work through each event in turn, dispatching it.  This is
             * slighly back to front, as the association between events and
             * handlers is imperfect: but simplicity wins here. */
            for (int i = 0; i < EVENT_TABLE_SIZE; i ++)
            {
                EVENT_TABLE & Event = EventTable[i];
                if (Event.Valid)
                {
                    /* Pick up the event and consume it. */
                    bool Occurred;
                    int Parameter;
                    THREAD_LOCK(this);
                    Occurred = Event.Occurred;
                    Parameter = Event.Parameter;
                    Event.Occurred = false;
                    THREAD_UNLOCK();
                    
                    /* Finally dispatch this event to all interested
                     * handlers -- if it actually occurred! */
                    if (Occurred)
                    {
                        for (int j = 0; j < HANDLER_TABLE_SIZE; j ++)
                        {
                            HANDLER_TABLE & Handler = HandlerTable[j];
                            if (Handler.EventId == Event.EventId)
                                Handler.Handler->OnEvent(Parameter);
                        }
                    }
                }
            }
        }
    }


#ifdef UNSAFE_PTHREAD_CANCEL
    /* We can't rely on the normal pthread_cancel() for thread termination
     * (as it can have the sorry side effect of breaking the Libera driver) 
     * -- but it is often sufficient to simply wake the thread up again. */
    void OnTerminate()
    {
        signal.Signal();
    }
#endif


    /* These two hard-wired limits can easily be updated as new events or
     * handlers are added. */
    enum {
        EVENT_TABLE_SIZE = 5,   // Number of distinct event ids handled
    };
    
    /* Table of events by event id: this is used to record the status of each
     * event. */
    struct EVENT_TABLE
    {
        bool Valid;             // Set if this entry is valid
        int EventId;            // Associated event 
        bool Occurred;          // Whether this event has occurred
        int Parameter;          // Merged event parameter
    };

    /* Handler dispatch table.  Again we're not being clever about this at
     * all: an array of event handlers, one array for each event type. */
    struct HANDLER_TABLE
    {
        int EventId;            // Event this handler is interested in
        I_EVENT * Handler;      // Handler interface to call
    };

    
    EVENT_TABLE EventTable[EVENT_TABLE_SIZE];
    HANDLER_TABLE HandlerTable[HANDLER_TABLE_SIZE];
    SEMAPHORE signal;
};



static EVENT_DISPATCHER * EventDispatcher = NULL;



/*****************************************************************************/
/*                                                                           */
/*                         Internal Event Receiver                           */
/*                                                                           */
/*****************************************************************************/



/* This thread receives the device driver event notifications from the
 * /dev/libera.event pipe and dispatches them to the appropriate event handler
 * thread. */

class EVENT_RECEIVER : public THREAD
{
public:
    EVENT_RECEIVER() : THREAD("EVENT_RECEIVER") {}
    
private:
    /* We connect directly to the event source and dispatch our events
     * ourselves.  The events are processed in a separate thread to ensure
     * that all events are actually seen, even if the consumer is too busy. */

    void Thread()
    {
        if (SetEventMask(EventDispatcher->EventMask()))
        {
            StartupOk();
            /* Run until we're forcibly killed from outside. */
            while (true)
            {
                int EventId, Param;
                if (ReadEvent(EventId, Param))
                    EventDispatcher->NotifyEvent(EventId, Param);
            }
        }
    }
};

static EVENT_RECEIVER * EventReceiver = NULL;



/*****************************************************************************/
/*                                                                           */
/*                           External Interface                              */
/*                                                                           */
/*****************************************************************************/


void RegisterTriggerEvent(I_EVENT &Event, PRIORITIES Priority)
{
    EventDispatcher->Register(Event, LIBERA_EVENT_TRIGGET, Priority);
}

void RegisterTriggerSetEvent(I_EVENT &Event, PRIORITIES Priority)
{
    EventDispatcher->Register(Event, LIBERA_EVENT_TRIGSET, Priority);
}

void RegisterPostmortemEvent(I_EVENT &Event, PRIORITIES Priority)
{
    EventDispatcher->Register(Event, LIBERA_EVENT_PM, Priority);
}

void RegisterInterlockEvent(I_EVENT &Event, PRIORITIES Priority)
{
    EventDispatcher->Register(Event, LIBERA_EVENT_INTERLOCK, Priority);
}


bool InitialiseEventReceiver()
{
    EventDispatcher = new EVENT_DISPATCHER();
    EventReceiver = new EVENT_RECEIVER();
    /* Enable the set of events to be supported: this needs to be done before
     * the event receiver thread is started.  Every event for which a
     * register function exist above should be enabled here. */
    EventDispatcher->EnableEvent(LIBERA_EVENT_TRIGGET);
    EventDispatcher->EnableEvent(LIBERA_EVENT_TRIGSET);
    EventDispatcher->EnableEvent(LIBERA_EVENT_PM);
    EventDispatcher->EnableEvent(LIBERA_EVENT_INTERLOCK);
    return
        EventDispatcher->StartThread()  &&
        EventReceiver->StartThread();
}


void TerminateEventReceiver()
{
    if (EventReceiver != NULL)
        EventReceiver->Terminate();
    if (EventDispatcher != NULL)
        EventDispatcher->Terminate();
}
