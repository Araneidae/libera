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


/* This code is concerned with receiving events from Libera and dispatching
 * them to interested parties. */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>

#include "hardware.h"
#include "thread.h"

#include "events.h"



/*****************************************************************************/
/*                                                                           */
/*                         Event Receiver List                               */
/*                                                                           */
/*****************************************************************************/

/* The following class implements a thread safe prioritised subscription
 * list.  Lowest priority entries are scheduled first. */

class RECEIVER_LIST
{
public:
    RECEIVER_LIST()
    {
        pthread_mutex_init(&ListMutex, NULL);
        List = NULL;
    }

    /* Note that subscribers can only *add* entries.  This simplifies this
     * class substantially! */
    void Subscribe(I_EVENT & iEvent, CSPI_EVENTMASK EventType, int Priority)
    {
        SUBSCRIPTION * Subscription = new SUBSCRIPTION;
        Subscription->iEvent = & iEvent;
        Subscription->EventType = EventType;
        Subscription->Priority = Priority;

        Lock();
        SUBSCRIPTION ** pList = &List;
        while (*pList != NULL  &&  (*pList)->Priority <= Priority)
            pList = & (*pList)->Next;
        
        Subscription->Next = *pList;
        *pList = Subscription;
        Unlock();
    }


    /* The following two methods support iterative walking of the
     * subscription list. */

    /* Start iteration. */
    void StartIteration()
    {
        Lock();
        Iterator = List;
        Unlock();
    }

    /* Return next subscriber, or returns false if end of list. */
    bool StepIteration(I_EVENT * &iEvent, CSPI_EVENTMASK &EventType)
    {
        if (Iterator == NULL)
            return false;
        else
        {
            iEvent    = Iterator->iEvent;
            EventType = Iterator->EventType;
            
            Lock();
            Iterator = Iterator->Next;
            Unlock();
            return true;
        }
    }
    
    
private:

    void Lock()
    {
        TEST_(pthread_mutex_lock, &ListMutex);
    }

    void Unlock()
    {
        TEST_(pthread_mutex_unlock, &ListMutex);
    }

    struct SUBSCRIPTION
    {
        SUBSCRIPTION * Next;
        I_EVENT * iEvent;
        CSPI_EVENTMASK EventType;
        int Priority;
    };

    SUBSCRIPTION * List;
    SUBSCRIPTION * Iterator;
    
    pthread_mutex_t ListMutex;
};



/*****************************************************************************/
/*                                                                           */
/*                         Event Receiver Thread                             */
/*                                                                           */
/*****************************************************************************/


/* The event receiver class encapsulates our event dispatching thread.  This
 * thread listens on the event device and dispatches events to interested
 * parties. */

class EVENT_RECEIVER : public THREAD
{
public:
    EVENT_RECEIVER()
    {
        /* Set up a mechanism for thread termination.
         * Actually, we shouldn't have to do it this way... */
        if (!TEST_(pipe, ThreadStop))
            /* Remember that we couldn't get started. */
            ThreadStop[0] = -1;
    }


    /* This triggers thread shutdown in the background. */
    void OnTerminate()
    {
        /* Tell the thread to stop. */
        int written;
        char stop = 0;
        TEST_IO(written, "Unable to write to stop pipe",
            write, ThreadStop[1], &stop, 1);
    }


    /* Register interest in a particular event type. */
    void Register(I_EVENT & Event, CSPI_EVENTMASK EventType, int Priority)
    {
        Receivers.Subscribe(Event, EventType, Priority);
    }

    
private:
    void Thread()
    {
        /* If we couldn't create the thread stop pipe then give up right
         * away, otherwise all is well. */
        if (ThreadStop[0] == -1)  return;
        StartupOk();
        
        int stop_fd = ThreadStop[0];
        int queue_fd = EventSelector();
        int max_fd = stop_fd > queue_fd ? stop_fd : queue_fd;
        while (true)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(stop_fd, &readfds);
            FD_SET(queue_fd, &readfds);
            int selected;
            if (TEST_IO(selected, "select failed",
                select, max_fd + 1, &readfds, NULL, NULL, NULL))
            {
                if (FD_ISSET(stop_fd, &readfds))
                    /* Stop request.  We could go and read it, but basically
                     * we stop right now. */
                    break;

                if (FD_ISSET(queue_fd, &readfds))
                    DispatchEvents();
            }
            else
                /* If select fails we might as well give up right now. */
                break;
        }
    }

    void DispatchEvents()
    {
        /* First discover which events need to be dispatched. */
        int IncomingEvents = ReadEvents();

        /* Work through all possibly interested parties and dispatch events
         * to all who wish to hear. */
        Receivers.StartIteration();
        I_EVENT * iEvent;
        CSPI_EVENTMASK EventId;
        while (Receivers.StepIteration(iEvent, EventId))
        {
            if (IncomingEvents & EventId)
                iEvent->OnEvent();
        }
    }


    /* Returns the next event from the event queue.
     *
     * Checks for multiple events and complains if more than one event
     * encountered.  This isn't quite right, but we do need to keep the event
     * queue clear, even if we're busy. */
    int ReadEvents()
    {
        int IncomingEvents = 0;
        CSPI_EVENTMASK Id;
        int Param;
        while (ReadOneEvent(Id, Param))
            /* Record this event. */
            IncomingEvents |= Id;
        return IncomingEvents;
    }


    /* These handles are the two ends of a pipe used to ask the thread to stop
     * processing. */
    int ThreadStop[2];

    /* Register queue.  This is a list of interested parties who will receive
     * notification of events. */
    RECEIVER_LIST Receivers;
};


static EVENT_RECEIVER * EventReceiver = NULL;


bool InitialiseEventReceiver()
{
    EventReceiver = new EVENT_RECEIVER();
    return EventReceiver->StartThread();
}


void TerminateEventReceiver()
{
    if (EventReceiver != NULL)
        EventReceiver->Terminate();
}


void RegisterTriggerEvent(I_EVENT &Event, int Priority)
{
    EventReceiver->Register(Event, CSPI_EVENT_TRIGGET, Priority);
}

void RegisterTriggerSetEvent(I_EVENT &Event, int Priority)
{
    EventReceiver->Register(Event, CSPI_EVENT_TRIGSET, Priority);
}

void RegisterPostmortemEvent(I_EVENT &Event, int Priority)
{
    EventReceiver->Register(Event, CSPI_EVENT_PM, Priority);
}

void RegisterInterlockEvent(I_EVENT &Event, int Priority)
{
    EventReceiver->Register(Event, CSPI_EVENT_INTERLOCK, Priority);
}
