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


/* Implementation of Booster data support. */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "drivers.h"
#include "publish.h"
#include "trigger.h"
#include "hardware.h"
#include "convert.h"
#include "support.h"

#include "slowAcquisition.h"



class SLOW_ACQUISITION
{
public:
    SLOW_ACQUISITION() 
    {
        Publish_longin("SA:A", ABCD.A);
        Publish_longin("SA:B", ABCD.B);
        Publish_longin("SA:C", ABCD.C);
        Publish_longin("SA:D", ABCD.D);
        Publish_ai("SA:X", X);
        Publish_ai("SA:Y", Y);
        Publish_ai("SA:S", S);
        Publish_ai("SA:Q", Q);

        /* Publish the trigger.  This trigger will be signalled whenever our
         * data has updated. */
        Publish_bi("SA:TRIG", Trigger);

        /* Start the slow acquisition thread. */
        ThreadRunning = true;
        pthread_t ThreadId;
        ThreadOk = TEST_(pthread_create, &ThreadId, NULL, StartThread, this);
    }


private:
    
    static void * StartThread(void * Context)
    {
        ((SLOW_ACQUISITION *) Context)->Thread();
        return NULL;
    }

    void Thread()
    {
        /* To do: synchronise with thread shutdown. */
        while (ThreadRunning)
        {
            if (ReadSlowAcquisition(ABCD))
            {
                LIBERA_ROW Row;
                Row[0] = ABCD.A;
                Row[1] = ABCD.B;
                Row[2] = ABCD.C;
                Row[3] = ABCD.D;
                ABCDtoXYQS(&Row, 1);
                
                X = nmTOmm(Row[4]);
                Y = nmTOmm(Row[5]);
                Q = nmTOmm(Row[6]);
                S = Row[7];
                Trigger.Ready();
            }
        }
    }

    bool ThreadOk;
    bool ThreadRunning;
    
    TRIGGER Trigger;

    SA_DATA ABCD;
    double X, Y, S, Q;
};



SLOW_ACQUISITION * SlowAcquisition = NULL;

bool InitialiseSlowAcquisition()
{
    SlowAcquisition = new SLOW_ACQUISITION();
    return true;
}

#if 0
void TerminateSlowAcquisition()
{
    if (SlowAcquisition != NULL)
        SlowAcquisition->Terminate();
}
#endif
