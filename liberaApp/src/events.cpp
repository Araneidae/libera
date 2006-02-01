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
    void Subscribe(I_EVENT & iEvent, int EventType, int Priority)
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
    bool StepIteration(I_EVENT * &iEvent, int &EventType)
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
        int EventType;
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

//#define PRINTF(args...) printf(args)
#define PRINTF(args...) 

/* The event receiver class encapsulates our event dispatching thread.  This
 * thread listens on the event device and dispatches events to interested
 * parties. */

class EVENT_RECEIVER
{
public:
    EVENT_RECEIVER()
    {
        ThreadRunning = false;
        ThreadStop[0] = -1;
    }

    /* Try to start the event receiver, reporting if anything balks. */
    bool Start()
    {
        /* First make sure we have a connection so we can tell the thread to
         * stop. */
        bool Ok = TEST_(pipe, ThreadStop);
        if (!Ok)
            /* Remember that we couldn't get started. */
            ThreadStop[0] = -1;

        /* Now try to start the thread. */
        ThreadRunning = Ok  &&
            OpenEventStream()  &&
            TEST_(pthread_create, &ThreadId, NULL, StartThread, this);
        if (ThreadRunning)
        {
#if 0
            /* Put the thread into a higher priority: we want the reading of
             * data from Libera to take priority! */
            sched_param Param;
            Param.__sched_priority = 20;
            TEST_(pthread_setschedparam, ThreadId, SCHED_RR, &Param);
#endif
//            printf("Started Event Receiver\n");
        }
            
        return ThreadRunning;
    }


    /* This triggers thread shutdown in the background.  This may be called
     * if thread startup failed! */
    void Shutdown()
    {
        if (ThreadRunning)
        {
            /* Tell the thread to stop and wait for it to complete. */
            int written;
            char stop = 0;
            TEST_IO(written, "Unable to write to stop pipe",
                write, ThreadStop[1], &stop, 1);
            TEST_(pthread_join, ThreadId, NULL);

//            printf("Event Receiver Done\n");
        }
        
        if (ThreadStop[0] != -1)
        {
            close(ThreadStop[0]);
            close(ThreadStop[1]);
        }
    }


    /* Register interest in a particular event type. */
    void Register(I_EVENT & Event, int EventType, int Priority)
    {
        Receivers.Subscribe(Event, EventType, Priority);
    }

    
private:

    /* This static routine simply dispatches the thread function to a proper
     * class member function. */
    static void * StartThread(void * Context)
    {
        ((EVENT_RECEIVER *) Context)->Thread();
        return NULL;
    }

    void Thread()
    {
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
                    DispatchEvent();
            }
            else
                /* If select fails we might as well give up right now. */
                break;
        }
    }

    void DispatchEvent()
    {
        LIBERA_EVENT_ID EventId;
        int param;
        if (ReadEvent(EventId, param))
        {
            PRINTF("{");
            I_EVENT * iEvent;
            int EventType;
            Receivers.StartIteration();
            while (Receivers.StepIteration(iEvent, EventType))
            {
                if (EventId == EventType)
                {
                    PRINTF("[");
                    iEvent->OnEvent();
                    PRINTF("]");
                }
            }
            PRINTF("}");
        }
        else
            printf("ReadEvent failed\n");
    }

    /* Returns the next event from the event queue.
     *
     * Checks for multiple events and complains if more than one event
     * encountered.  This isn't quite right, but we do need to keep the event
     * queue clear, even if we're busy. */
    bool ReadEvent(LIBERA_EVENT_ID &Id, int &Param)
    {
//        printf(".");
        int EventCount = 0;
        while (ReadOneEvent(Id, Param))
        {
            EventCount ++;
            if (Id != LIBERA_EVENT_TRIGGER)
                printf("Event %d, %d received\n", Id, Param);
        }
        if (EventCount > 1)
//            printf("Missed %d events!\n", EventCount - 1);
            ;
        else if (EventCount == 0)
            printf("Odd: nothing to read?\n");
        return EventCount >= 1;
    }


    /* These handles are the two ends of a pipe used to ask the thread to stop
     * processing. */
    int ThreadStop[2];
    /* Id for event worker thread. */
    pthread_t ThreadId;
    /* Set iff thread actually got started. */
    bool ThreadRunning;

    /* Register queue.  This is a list of interested parties who will receive
     * notification of events. */
    RECEIVER_LIST Receivers;
};


EVENT_RECEIVER * EventReceiver = NULL;


bool InitialiseEventReceiver()
{
    EventReceiver = new EVENT_RECEIVER();
    return EventReceiver->Start();
}


void TerminateEventReceiver()
{
    if (EventReceiver != NULL)
        EventReceiver->Shutdown();
}


void RegisterEvent(I_EVENT &Event, int Priority)
{
    EventReceiver->Register(Event, LIBERA_EVENT_TRIGGER, Priority);
}
