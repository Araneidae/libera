/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2006  Michael Abbott, Diamond Light Source Ltd.
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


/* Interlock management. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
 
#include <dbFldTypes.h>
 
#include "drivers.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"

#include "interlock.h"




/* Interlock configuration values with sensible defaults. */

/* Interlock mode: 0 => disabled, 1 => always enabled, 3 => gain dependent
 * interlock. */
static int InterlockMode = 0;
/* Interlock position window. */
static int MinX = -1000000;
static int MaxX = 1000000;
static int MinY = -1000000;
static int MaxY = 1000000;
/* Interlock ADC overflow limits. */
static int OverflowLimit = 1900;
static int OverflowTime = 5;
/* Interlock gain level limit. */
static int GainLimit = -40;


static void UpdateInterlock()
{
    WriteInterlockParameters(
        (CSPI_ILKMODE) InterlockMode, MinX, MaxX, MinY, MaxY,
        OverflowLimit, OverflowTime, GainLimit);
}


#define PUBLISH_INTERLOCK(Name, recout, Value) \
    PUBLISH_CONFIGURATION(Name, recout, Value, UpdateInterlock) 



class INTERLOCK_EVENT : public I_EVENT
{
public:
    INTERLOCK_EVENT() :
        InterlockTrigger(true)
    {
        Publish_bi("IL:TRIG", InterlockTrigger);
        RegisterInterlockEvent(*this, PRIORITY_IL);
    }

    void OnEvent()
    {
        InterlockTrigger.Ready();
    }
    
private:
    TRIGGER InterlockTrigger;
};



bool InitialiseInterlock()
{
    PUBLISH_INTERLOCK("IL:MODE", mbbo, InterlockMode);
    PUBLISH_INTERLOCK("IL:MINX", ao, MinX);
    PUBLISH_INTERLOCK("IL:MAXX", ao, MaxX);
    PUBLISH_INTERLOCK("IL:MINY", ao, MinY);
    PUBLISH_INTERLOCK("IL:MAXY", ao, MaxY);
    PUBLISH_INTERLOCK("IL:GAIN", longout, GainLimit);
    PUBLISH_INTERLOCK("IL:OVER", longout, OverflowLimit);
    PUBLISH_INTERLOCK("IL:TIME", longout, OverflowTime);
    UpdateInterlock();

    new INTERLOCK_EVENT();
    
    return true;
}

