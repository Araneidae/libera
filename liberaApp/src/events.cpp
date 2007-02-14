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
#include <fcntl.h>
#include <signal.h>
#include <sys/select.h>
#include <pthread.h>

#include "eventd.h"     // for LIBERA_SIGNAL
#include "hardware.h"
#include "thread.h"

#include "events.h"


static int EventSelector();
static bool ReadOneEvent(CSPI_EVENTMASK &Id, int &Param);



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
    EVENT_RECEIVER() : THREAD("EVENT_RECEIVER") {}


    /* Register interest in a particular event type. */
    void Register(I_EVENT & Event, CSPI_EVENTMASK EventType, int Priority)
    {
        Receivers.Subscribe(Event, EventType, Priority);
    }

    
private:
    void Thread()
    {
        StartupOk();
        
        int queue_fd = EventSelector();
        int max_fd = queue_fd;
        while (Running())
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(queue_fd, &readfds);
            int selected;
            if (TEST_IO(selected, "select failed",
                select, max_fd + 1, &readfds, NULL, NULL, NULL))
            {
                if (FD_ISSET(queue_fd, &readfds))
                    DispatchEvents();
            }
            else
            {
                printf("select aborting\n");
                /* If select fails we might as well give up right now. */
                break;
            }
        }
    }

    void OnTerminate() { }

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


    /* Register queue.  This is a list of interested parties who will receive
     * notification of events. */
    RECEIVER_LIST Receivers;
};




/*****************************************************************************/
/*                                                                           */
/*                        Direct Event Connection                            */
/*                                                                           */
/*****************************************************************************/

/* Libera low level device, used to received event notification. */
static int libera_event = -1;
static int event_source = -1;

/* Reads one event from the event queue and decode it into an event id and
 * its associated parameter information.  The caller should empty the queue
 * by calling this routine until it returns false before processing any
 * events. */

static bool ReadOneEvent(CSPI_EVENTMASK &Id, int &Param)
{
    libera_event_t event;
    ssize_t bytes_read = read(libera_event, &event, sizeof(event));
    if (bytes_read == 0  ||  (bytes_read == -1  &&  errno == EAGAIN))
        /* Nothing to read this time: the queue is empty. */
        return false;
    else if (bytes_read > 0)
    {
        if (bytes_read < (ssize_t) sizeof(event))
        {
            /* Oops.  Half an event?  This is going to be trouble!  We have
             * no good way to recover from this.  We should, in principle,
             * keep on trying to read until a full event arrives...
             *     Fortunately this never seems to happen. */
            printf("Incomplete event read: %d bytes read", bytes_read);
            return false;
        }
        else
        {
            /* Good.  Return the event. */
            Id = (CSPI_EVENTMASK) event.id;
            Param = event.param;
            return true;
        }
    }
    else
    {
        perror("Error reading event queue");
        return false;
    }
}



/* This thread works to receive the CSPI events synchronously. */

class LEVENTD_THREAD : public THREAD
{
public:
    LEVENTD_THREAD() : THREAD("LEVENTD_THREAD") {}
    
private:
    void Thread()
    {
        int EventMask =
            CSPI_EVENT_TRIGGET |
            CSPI_EVENT_TRIGSET | 
            CSPI_EVENT_PM |
            CSPI_EVENT_INTERLOCK;
        struct sigaction CspiAction;
        sigset_t EnableSet;
        bool Ok =
            /* Request CSPI events. */
            ConfigureEventCallback(EventMask, CspiSignal)  &&
            /* Now completely reprogram the signal handler so that we can
             * handle the LIBERA_SIGNAL synchronously.  First pick up the
             * CSPI action and then reset the action to default so we can
             * receive it synchronously. */
            TEST_(sigaction, LIBERA_SIGNAL, NULL, &CspiAction)  &&
            /* Enable delivery of the LIBERA_SIGNAL signal, to this thread
             * only: this is required for CSPI event dispatching. */
            TEST_(sigemptyset, &EnableSet)  &&
            TEST_(sigaddset, &EnableSet, LIBERA_SIGNAL);
            
        
        /* If all is well then report success and then just sit and wait for
         * shutdown: all the real work now happens in the background. */
        if (Ok)
        {
            StartupOk();

            int Signal;
            siginfo_t SigInfo;
            while (TEST_IO(Signal, "Error receiving signal",
                    sigwaitinfo, &EnableSet, &SigInfo))
            {
                if (Signal == LIBERA_SIGNAL)
                    CspiAction.sa_sigaction(Signal, &SigInfo, NULL);
                else
                    printf("Unexpected signal %d\n", Signal);
            }
        }

    }

    /* This is the signal handler called from CSPI.  This is now synchronous,
     * so we can use our thread dispatching mechanisms of choice to
     * redispatch the events internally. */
    static int CspiSignal(CSPI_EVENT *Event)
    {
        int written = write(event_source, &Event->hdr, sizeof(Event->hdr));
        if (written != sizeof(Event))
            /* Well? */ ;
        return 1;
    }

    CSPIHCON EventSource;
};


static LEVENTD_THREAD * LeventdThread = NULL;

/* To be called on initialisation to enable delivery of events. */

static bool OpenEventStream()
{
    int Pipes[2];
    if (TEST_(pipe, Pipes)  &&
        TEST_(fcntl, Pipes[0], F_SETFL, O_NONBLOCK))
    {
        libera_event = Pipes[0];
        event_source = Pipes[1];
        LeventdThread = new LEVENTD_THREAD();
        return LeventdThread->StartThread();
    }
    else
        return false;
}


/* Because the event consumer needs to use select() to wait for event
 * delivery we need to break encapsulation here and expose the actual handle
 * used to receive events. */

static int EventSelector()
{
    return libera_event;
}


/* Called on termination to cancel event delivery. */

static void CloseEventStream()
{
    if (LeventdThread != NULL)
        LeventdThread->Terminate();
    if (event_source != -1)  close(event_source);
    if (libera_event != -1)  close(libera_event);
}




/*****************************************************************************/
/*                                                                           */
/*                           External Interface                              */
/*                                                                           */
/*****************************************************************************/


static EVENT_RECEIVER * EventReceiver = NULL;


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



bool InitialiseEventReceiver()
{
    /* Finally start receiving events. */
    OpenEventStream();
    EventReceiver = new EVENT_RECEIVER();
    return EventReceiver->StartThread();
}


void TerminateEventReceiver()
{
    CloseEventStream();
    if (EventReceiver != NULL)
        EventReceiver->Terminate();
}
