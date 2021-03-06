/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2006-2011 Michael Abbott, Diamond Light Source Ltd.
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
#include <unistd.h>
#include <string.h>
#include <stddef.h>
#include <pthread.h>
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"

#include "interlock.h"


/* Every interlock control variable is handled in the same way: the value is
 * updated and then UpdateInterlock() is called to ensure the interlock state
 * is correctly managed. */
#define PUBLISH_INTERLOCK(record, Name, Value) \
    PUBLISH_CONFIGURATION(record, Name, Value, LockedWriteInterlockState)



/* Interlock configuration values with sensible defaults. */

/* Interlock position window. */
static int MinX = -1000000;     // +- 1 mm
static int MaxX =  1000000;
static int MinY = -1000000;
static int MaxY =  1000000;
/* Secondary interlock window. */
static int MinX2 = -1000000;     // +- 1 mm
static int MaxX2 =  1000000;
static int MinY2 = -1000000;
static int MaxY2 =  1000000;
/* Current interlock computed from interlocks above and interlock window. */
static int currentMinX = -1000000;     // +- 1 mm
static int currentMaxX =  1000000;
static int currentMinY = -1000000;
static int currentMaxY =  1000000;

/* Interlock position offset: these need to adjust the position of the window
 * to take account of Golden Orbit offsets. */
static int OffsetX = 0;
static int OffsetY = 0;

/* Current threshold for enabling interlock. */
static int InterlockAutoOnCurrent = 1000000;    // 10mA
static int InterlockAutoOffCurrent = 0;         // 0mA
/* Last current reading. */
static int CurrentCurrent = 0;

/* Interlock ADC overflow limits. */
static bool OverflowEnable = false;
static int OverflowLimit = 30000;
static int OverflowTime = 5;

/* Global interlock override: if this BPM is disabled then the interlock is
 * ignored and never enabled. */
static bool GlobalBpmEnable = true;

/* The master interlock enable tracks the overall state of the interlock.
 * This is forced true when current is >ION, forced false when current <IOFF
 * (except during the interlock holdoff interval) and when current is between
 * these two values can be manually controlled. */
static bool MasterInterlockEnable = false;

/* The enable readback is used to keep the IL:ENABLE control in step with the
 * MasterInterlockEnable variable above masked with GlobalBpmEnable. */
static READBACK_bool * EnableReadback = NULL;

/* The interlock test mode is used to force the interlock to be dropped.  This
 * mode overrides all other activity. */
static bool InterlockTestMode = false;



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
static int InterlockHoldoffCount = 1;   // Not so clear what's suitable

static int InterlockHoldoff = 3;

/* Interlock ADC IIR filter constant.  The interlock ADC overflow detection is
 * filtered by an IIR with constant factor 2^-K determined by this setting. */
static int InterlockIIR_K = 0;
/* Interlock X/Y IIR filter constant.  The interlock X/Y position is filtered by
 * an IIR with constant factor (1+K)/256 determined by this setting. */
static int InterlockXYIIR_K = 255;


/* We're going to need to use a mutex, as there are two possible threads
 * coming through here and interactions between them need to be guarded.  One
 * thread will be the main EPICS processing thread (causing most configuration
 * changes, including calling TemporaryMaskInterlock()), and the other thread
 * is the slow acquisition update thread. */
static pthread_mutex_t InterlockMutex = PTHREAD_MUTEX_INITIALIZER;

static void Lock()         { TEST_0(pthread_mutex_lock(&InterlockMutex)); }
static void Unlock(void *) { TEST_0(pthread_mutex_unlock(&InterlockMutex)); }

#define LOCK()      Lock(); pthread_cleanup_push(Unlock, NULL)
#define UNLOCK()    pthread_cleanup_pop(true)



/* Programs the interlock hardware as appropriate.  This should be called
 * inside the lock. */

static void WriteInterlockState()
{
    if (InterlockTestMode)
    {
        /* In interlock test mode we unconditionally force the interlock to
         * be dropped by writing an impossible window and overflow limit. */
        WriteInterlockParameters(LIBERA_ILK_ENABLE, 0, 0, 0, 0, 1, 1, 0);
        WriteSecondaryInterlockParameters(0, 0, 0, 0);
    }
    else
    {
        LIBERA_ILKMODE InterlockMode;
        if (!GlobalBpmEnable)
            /* In BPM disable state the interlock is unconditionally disabled.
             * The variable GlobalBpmEnable tracks CF:ENABLED. */
            InterlockMode = LIBERA_ILK_DISABLE;
        else if (InterlockHoldoff > 0)
            /* In holdoff mode the interlock is unconditionally disabled.
             * This masks out interlocks after the attenuators have changed. */
            InterlockMode = LIBERA_ILK_DISABLE;
        else if (MasterInterlockEnable)
            /* In normal enabled mode the interlock is unconditionally
             * enabled. */
            InterlockMode = LIBERA_ILK_ENABLE;
        else if (OverflowEnable)
            /* In overflow detection mode (with the master interlock disabled)
             * we use a tricksy hack to enable ADC overflow detection while
             * disabling position interlock: we enable gain dependent
             * interlock mode, which enables position interlocking only when
             * the "gain" is above a certain threshold, and we simultaneously
             * set an impossibly high gain threshold. */
            InterlockMode = LIBERA_ILK_ENABLE_GAINDEP;
        else
            /* If none of the above apply then the interlock is disabled. */
            InterlockMode = LIBERA_ILK_DISABLE;

        WriteInterlockParameters(
            InterlockMode,
            MinX - OffsetX, MaxX - OffsetX, MinY - OffsetY, MaxY - OffsetY,
            OverflowLimit, OverflowTime, 0);
        WriteSecondaryInterlockParameters(
            MinX2 - OffsetX, MaxX2 - OffsetX, MinY2 - OffsetY, MaxY2 - OffsetY);
    }
}



/* Sets the underlying interlock enable state to the requested value, taking
 * auto on/off actions into account.  This must be called from within a
 * lock. */

static void UpdateInterlockEnable(bool SetEnable)
{
    /* Update the interlock state according to the observed current together
     * with the requested setting. */
    MasterInterlockEnable = SetEnable;
    if (CurrentCurrent > InterlockAutoOnCurrent)
        MasterInterlockEnable = true;
    if (CurrentCurrent < InterlockAutoOffCurrent)
        MasterInterlockEnable = false;
    if (!GlobalBpmEnable)
        MasterInterlockEnable = false;

    /* Ensure the interlock enabled control correctly reflects the newly
     * calculated state. */
    EnableReadback->Write(MasterInterlockEnable);
    WriteInterlockState();
}



/* This is called whenever any part of the persistent configuration of the
 * interlock changes: this is called from EPICS.  All we need to do is ensure
 * that the interlock is configured. */

static void LockedWriteInterlockState()
{
    LOCK();
    WriteInterlockState();
    UNLOCK();
}


/* Called in response to a change in the ENABLE user control.  Note that the
 * underlying READBACK mechanism will ensure that this routine is only called
 * if the IL:ENABLE has actually changed. */

static bool LockedUpdateInterlockEnable(bool SetEnable)
{
    LOCK();
    UpdateInterlockEnable(SetEnable);
    UNLOCK();
    return true;
}


/* This is called from the SA thread each time the current is sampled, at
 * approximately 10Hz.  If the current goes over the interlock enable
 * threshold then we turn interlocking on.  This routine is also used to time
 * out the interlock holdoff state. */

void NotifyInterlockCurrent(int Current)
{
    LOCK();

    CurrentCurrent = Current;
    if (InterlockHoldoff > 0)
        /* Count off the interlock holdoff.  Ignore the current during this
         * holdoff period. */
        InterlockHoldoff -= 1;
    else
        /* During normal operation just refresh the master interlock enable
         * state: this will take account of any current on/off effects. */
        UpdateInterlockEnable(MasterInterlockEnable);

    UNLOCK();
}


/* This is called when a change which can cause a position or current glitch
 * is about to be made.  We temporarily disable interlocking to prevent the
 * position glitch from dropping the interlock. */

void HoldoffInterlock()
{
    LOCK();

    /* Figure out which holdoff we need. */
    if (MasterInterlockEnable  ||  OverflowEnable)
        /* Interlock is currently (potentially) enabled.  Disable it while we
         * perform the update. */
        InterlockHoldoff = InterlockHoldoffCount;
    else
        /* Interlock is not currently enabled, so all we need to watch out
         * for is the current spike. */
        InterlockHoldoff = CurrentHoldoffCount;

    WriteInterlockState();
    UNLOCK();
}



/* Called when the global BPM enable state changes. */

bool NotifyInterlockBpmEnable(bool Enabled)
{
    LOCK();
    GlobalBpmEnable = Enabled;
    UpdateInterlockEnable(false);
    UNLOCK();
    return true;
}



/* Called when the origin offset may have changed. */

void NotifyInterlockOffset(int NewOffsetX, int NewOffsetY)
{
    LOCK();
    OffsetX = NewOffsetX;
    OffsetY = NewOffsetY;
    WriteInterlockState();
    UNLOCK();
}


/* This class receives the interlock event (used to indicate that the
 * interlock is currently dropped) and communicates it to the epics layer. */

class INTERLOCK_EVENT : public I_EVENT
{
public:
    INTERLOCK_EVENT()
    {
        Publish_longin("IL:RAW_REASON", InterlockReason);
        Interlock.Publish("IL");

        RegisterInterlockEvent(*this, PRIORITY_IL);
    }

    void OnEvent(int ReasonMask)
    {
        Interlock.Wait();
        InterlockReason = ReasonMask;
        Interlock.Ready();
    }

private:
    int InterlockReason;
    INTERLOCK Interlock;
};


static bool ReadInterlockWindow(int &window)
{
    uint32_t status;
    bool ok = ReadInterlockStatus(status);
    if (ok)
    {
        window = (status >> 5) & 1;
        if (window)
        {
            /* window == 1 => Secondary interlock active. */
            currentMinX = MinX2;
            currentMaxX = MaxX2;
            currentMinY = MinY2;
            currentMaxY = MaxY2;
        }
        else
        {
            /* window == 0 => Primary interlock active. */
            currentMinX = MinX;
            currentMaxX = MaxX;
            currentMinY = MinY;
            currentMaxY = MaxY;
        }
    }
    return ok;
}


static void SetInterlockIIR_K()
{
    WriteInterlockIIR_K(InterlockIIR_K);
}


static void SetInterlockXYIIR_K()
{
    WriteInterlockXYIIR_K(InterlockXYIIR_K);
}


bool InitialiseInterlock()
{
    /* Interlock window plus secondary interlock. */
    PUBLISH_INTERLOCK(ao,  "IL:MINX",     MinX);
    PUBLISH_INTERLOCK(ao,  "IL:MAXX",     MaxX);
    PUBLISH_INTERLOCK(ao,  "IL:MINY",     MinY);
    PUBLISH_INTERLOCK(ao,  "IL:MAXY",     MaxY);
    PUBLISH_INTERLOCK(ao,  "IL:MINX2",    MinX2);
    PUBLISH_INTERLOCK(ao,  "IL:MAXX2",    MaxX2);
    PUBLISH_INTERLOCK(ao,  "IL:MINY2",    MinY2);
    PUBLISH_INTERLOCK(ao,  "IL:MAXY2",    MaxY2);
    /* Current threshold at which the interlock is automatically triggered. */
    PUBLISH_INTERLOCK(ao,  "IL:ION",      InterlockAutoOnCurrent);
    PUBLISH_INTERLOCK(ao,  "IL:IOFF",     InterlockAutoOffCurrent);
    /* Overflow detection configuration. */
    PUBLISH_INTERLOCK(bo,  "IL:OVERFLOW", OverflowEnable);
    PUBLISH_INTERLOCK(longout, "IL:OVER", OverflowLimit);
    PUBLISH_INTERLOCK(longout, "IL:TIME", OverflowTime);
    /* Interlock testing. */
    PUBLISH_FUNCTION_OUT(bo,  "IL:TEST",
        InterlockTestMode, LockedWriteInterlockState);
    PUBLISH_FUNCTION_IN(mbbi, "IL:WINDOW", ReadInterlockWindow);

    Publish_ai("IL:MINX", currentMinX);
    Publish_ai("IL:MAXX", currentMaxX);
    Publish_ai("IL:MINY", currentMinY);
    Publish_ai("IL:MAXY", currentMaxY);

    /* The interlock enable is dynamic state. */
    EnableReadback = PUBLISH_READBACK(bi, bo, "IL:ENABLE",
        false, LockedUpdateInterlockEnable);

    PUBLISH_CONFIGURATION(longout, "IL:HOLDOFF",
        InterlockHoldoffCount, NULL_ACTION);
    PUBLISH_CONFIGURATION(longout, "IL:IHOLDOFF",
        CurrentHoldoffCount, NULL_ACTION);
    PUBLISH_CONFIGURATION(longout, "IL:IIRK",
        InterlockIIR_K, SetInterlockIIR_K);
    PUBLISH_CONFIGURATION(longout, "IL:XY:IIRK",
        InterlockXYIIR_K, SetInterlockXYIIR_K);

    new INTERLOCK_EVENT();

    SetInterlockIIR_K();
    LockedWriteInterlockState();

    return true;
}
