/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2006-2007 Michael Abbott, Diamond Light Source Ltd.
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
#include <pthread.h>
 
#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "configure.h"

#include "interlock.h"


/* Every interlock control variable is handled in the same way: the value is
 * updated and then UpdateInterlock() is called to ensure the interlock state
 * is correctly managed. */
#define PUBLISH_INTERLOCK(record, Name, Value) \
    PUBLISH_CONFIGURATION(record, Name, Value, EpicsUpdateInterlock) 



/* Interlock configuration values with sensible defaults. */

/* Interlock position window. */
static int MinX = -1000000;     // +- 1 mm
static int MaxX =  1000000;
static int MinY = -1000000;
static int MaxY =  1000000;
/* Interlock position offset: these need to adjust the position of the window
 * to take account of Golden Orbit offsets. */
static int OffsetX = 0;
static int OffsetY = 0;

/* Current threshold for enabling interlock. */
static int InterlockAutoOnCurrent = 1000000;    // 10mA
static int InterlockAutoOffCurrent = 0;         // 0mA
static bool CurrentOverThreshold = false;
static bool CurrentUnderThreshold = false;

/* Interlock ADC overflow limits. */
static bool OverflowEnable = false;
static int OverflowLimit = 1900;
static int OverflowTime = 5;

/* Global interlock override: if this BPM is disabled then the interlock is
 * ignored and never enabled. */
static bool GlobalBpmEnable = true;
/* Master interlock enable: can be reset, but only if current is below
 * threshold. */
static bool MasterInterlockEnable = false;
static READBACK_bool * EnableReadback = NULL;



/* The interlock holdoff mechanism is required to ensure that when we change
 * the attenuators we don't also affect the state of the interlock: in
 * particular, we need to take care not to drop the interlock!
 *
 * It's quite important here that we mask out interlocks before *any* part of
 * the new attenuation value is written: there are two parts of the system
 * that are affected by this:
 *
 * 1. Changing the attenuators will cause a glitch in position: this can
 *    cause the interlock to be dropped if we don't mask it out first.
 *
 * 2. Changing the attenuators will cause a glitch in the observed
 *    current: this can cause the interlocks to be enabled unexpectedly
 *    (and thus dropped).
 *
 * This is managed by means of the InterlockHoldoff count which is used to
 * disable interlocks while attenuators are changed.
 *
 * At present the strategies are rather experimental.  The code is structured
 * to allow a delay between disabling the interlock and updating the
 * attenuators, but this is probably not necessary.  On the other hand, the
 * delay before the interlock is updated again is more of a problem.
 *
 * We currently support two holdoff delays, one for use when there is no
 * interlock used to guard the current, and a different delay for use when
 * interlock is enabled.  This second delay is currently programmable. */

static int CurrentHoldoffCount = 3;     // 300ms seems ample for this
static int InterlockHoldoffCount = 5;   // Not so clear what's suitable

static int InterlockHoldoff = 3;


/* We're going to need to use a mutex, as there are two possible threads
 * coming through here and interactions between them need to be guarded.  One
 * thread will be the main EPICS processing thread (causing most configuration
 * changes, including calling TemporaryMaskInterlock()), and the other thread
 * is the slow acquisition update thread. */
pthread_mutex_t InterlockMutex = PTHREAD_MUTEX_INITIALIZER;

static void Lock()   { TEST_(pthread_mutex_lock,   &InterlockMutex); }
static void Unlock() { TEST_(pthread_mutex_unlock, &InterlockMutex); }



/* Programs the interlock hardware as appropriate.  This should be called
 * inside the lock. */

static void WriteInterlockState()
{
    CSPI_ILKMODE InterlockMode;
    if (!GlobalBpmEnable)
        /* In BPM disable state the interlock is unconditionally disabled. */
        InterlockMode = CSPI_ILK_DISABLE;
    else if (InterlockHoldoff > 0)
        /* In holdoff mode the interlock is unconditionally disabled. */
        InterlockMode = CSPI_ILK_DISABLE;
    else if (MasterInterlockEnable)
        /* In normal enabled mode the interlock is unconditionally enabled. */
        InterlockMode = CSPI_ILK_ENABLE;
    else if (OverflowEnable)
        /* In overflow detection mode (with the master interlock disabled) we
         * use a tricksy hack to enable ADC overflow detection while
         * disabling position interlock: we enable gain dependent interlock
         * mode, which enables position interlocking only when the "gain" is
         * above a certain threshold, and we simultaneously set an impossibly
         * high gain threshold. */
        InterlockMode = CSPI_ILK_ENABLE_GAINDEP;
    else
        /* If none of the above apply then the interlock is disabled. */
        InterlockMode = CSPI_ILK_DISABLE;

    WriteInterlockParameters(
        InterlockMode,
        MinX - OffsetX, MaxX - OffsetX, MinY - OffsetY, MaxY - OffsetY,
        OverflowLimit, OverflowTime, 0);
}



/* This is called whenever any part of the persistent configuration of the
 * interlock changes: this is called from EPICS.  All we need to do is ensure
 * that the interlock is configured. */

static void EpicsUpdateInterlock()
{
    Lock();
    WriteInterlockState();
    Unlock();
}


/* Called in response to a change in the ENABLE user control.  We compute
 * what the value *should* be, and if it differs the value is written back.
 * Care needs to be taken to avoid processing loops, as any call to
 * EnableReadback.Write() will be reflected right back here! */

static void UpdateInterlockEnable(bool SetEnable)
{
    Lock();

    /* Compute the proper enable from the requested new state together with
     * other settings that affect the state. */
    MasterInterlockEnable = SetEnable;
    if (!GlobalBpmEnable)
        /* During BPM disable mode the enable state cannot be set. */
        MasterInterlockEnable = false;
    else if (CurrentOverThreshold)
        /* If the current is over limit the enable cannot be reset. */
        MasterInterlockEnable = true;
    else if (CurrentUnderThreshold)
        /* If the current is under limit enable cannot be set. */
        MasterInterlockEnable = false;

    /* Ensure that the control is in agreement with our calculated setting
     * and update the true interlock setting. */
    EnableReadback->Write(MasterInterlockEnable);
    WriteInterlockState();
    
    Unlock();
}


/* This is called each time the current is sampled, at approximately 10Hz.  If
 * the current goes over the interlock enable threshold then we turn
 * interlocking on.  This routine is also used to time out the interlock
 * holdoff state. */

void NotifyInterlockCurrent(int Current)
{
    Lock();

    if (InterlockHoldoff > 0)
    {
        /* Count off the interlock holdoff.  Ignore the current during this
         * holdoff period. */
        InterlockHoldoff -= 1;
        if (InterlockHoldoff == 0)
            /* When the interlock has finally expired update the interlock
             * state. */
            WriteInterlockState();
    }
    else if (GlobalBpmEnable)
    {
        /* Normal operation: check for current over threshold, and enable the
         * interlock if exceeded. */
        CurrentOverThreshold  = Current > InterlockAutoOnCurrent;
        CurrentUnderThreshold = Current < InterlockAutoOffCurrent;

        bool OldMasterInterlockEnable = MasterInterlockEnable;
        if (CurrentOverThreshold)   MasterInterlockEnable = true;
        if (CurrentUnderThreshold)  MasterInterlockEnable = false;
        if (OldMasterInterlockEnable != MasterInterlockEnable)
        {
            EnableReadback->Write(MasterInterlockEnable);
            WriteInterlockState();
        }
    }

    Unlock();
}


/* This is called when the attenuators are about to be changed.  We
 * temporarily disable interlocking to prevent the position glitch (which
 * follows from setting the interlock) from dropping the interlock. */

void InterlockedUpdateAttenuation(int NewAttenuation)
{
    Lock();

    /* Figure out which holdoff we need. */
    if (MasterInterlockEnable  ||  OverflowEnable)
    {
        /* Interlock is currently (potentially) enabled.  Disable it while we
         * perform the update. */
        InterlockHoldoff = InterlockHoldoffCount;
        WriteInterlockState();
    }
    else
        /* Interlock is not currently enabled, so all we need to watch out
         * for is the current spike. */
        InterlockHoldoff = CurrentHoldoffCount;
    
    Unlock();

    /* The interlock is now off, so just update the attenuators now. */
    UpdateAttenuation(NewAttenuation);
}



/* This class receives the interlock event (used to indicate that the
 * interlock is currently dropped) and communicates it to the epics layer. */

class INTERLOCK_EVENT : public I_EVENT
{
public:
    INTERLOCK_EVENT(const char * Name) :
        InterlockTrigger(true)
    {
        Publish_bi(Name, InterlockTrigger);
        RegisterInterlockEvent(*this, PRIORITY_IL);
    }

    void OnEvent(int ReasonMask)
    {
//        printf("Interlock reason: %02X\n", ReasonMask);
        InterlockTrigger.Ready();
    }
    
private:
    TRIGGER InterlockTrigger;
};




/* Called when the global BPM enable state changes. */

void NotifyInterlockBpmEnable(bool Enabled)
{
    GlobalBpmEnable = Enabled;
    EnableReadback->Write(false);
}



/* Called when the golden orbit offset may have changed. */

void NotifyInterlockOffset(int NewOffsetX, int NewOffsetY)
{
    OffsetX = NewOffsetX;
    OffsetY = NewOffsetY;
    EpicsUpdateInterlock();
}


bool InitialiseInterlock()
{
    /* Interlock window. */
    PUBLISH_INTERLOCK(ao,  "IL:MINX",     MinX);
    PUBLISH_INTERLOCK(ao,  "IL:MAXX",     MaxX);
    PUBLISH_INTERLOCK(ao,  "IL:MINY",     MinY);
    PUBLISH_INTERLOCK(ao,  "IL:MAXY",     MaxY);
    /* Current threshold at which the interlock is automatically triggered. */
    PUBLISH_INTERLOCK(ao,  "IL:ION",      InterlockAutoOnCurrent);
    PUBLISH_INTERLOCK(ao,  "IL:IOFF",     InterlockAutoOffCurrent);
    /* Overflow detection configuration. */
    PUBLISH_INTERLOCK(bo,  "IL:OVERFLOW", OverflowEnable);
    PUBLISH_INTERLOCK(longout, "IL:OVER", OverflowLimit);
    PUBLISH_INTERLOCK(longout, "IL:TIME", OverflowTime);

    /* The interlock enable is dynamic state. */
    EnableReadback = PUBLISH_READBACK(bi, bo, "IL:ENABLE",
        false, UpdateInterlockEnable);

    PUBLISH_CONFIGURATION(longout, "IL:HOLDOFF", 
        InterlockHoldoffCount, NULL_ACTION);
    PUBLISH_CONFIGURATION(longout, "IL:IHOLDOFF", 
        CurrentHoldoffCount, NULL_ACTION);
    
    new INTERLOCK_EVENT("IL:TRIG");

    EpicsUpdateInterlock();

    return true;
}
