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


/* Implementation of 10Hz "slow acquisition" data. */

#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "publish.h"
#include "trigger.h"
#include "hardware.h"
#include "convert.h"
#include "thread.h"
#include "waveform.h"

#include "slowAcquisition.h"


class SLOW_ACQUISITION : public THREAD
{
public:
    SLOW_ACQUISITION()
    {
        Publish_ABCD("SA", ABCD);
        Publish_XYQS("SA", XYQS);

        /* Publish the trigger.  This trigger will be signalled whenever our
         * data has updated. */
        Interlock.Publish("SA");
    }

private:
    void Thread()
    {
        StartupOk();

        /* We simply run until asked to stop.  Unfortunately we have no way
         * to interrupt ReadSlowAcquisition(), so we might lock up if that
         * stops responding. */
        while (Running())
        {
            Interlock.Wait();
            if (ReadSlowAcquisition(ABCD))
                ABCDtoXYQSmm(ABCD, XYQS);
            Interlock.Ready();
        }
    }
    
    INTERLOCK Interlock;

    ABCD_ROW ABCD;
    XYQSmm_ROW XYQS;
};



static SLOW_ACQUISITION * SlowAcquisition = NULL;

bool InitialiseSlowAcquisition()
{
    SlowAcquisition = new SLOW_ACQUISITION();
    return SlowAcquisition->StartThread();
}


void TerminateSlowAcquisition()
{
    if (SlowAcquisition != NULL)
        SlowAcquisition->Terminate();
}
