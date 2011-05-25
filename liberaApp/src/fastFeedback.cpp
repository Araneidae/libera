/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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


/* Support for configuration access to fast feedback control registers.
 * Diamond Light Source specific code. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"
#include "versions.h"

#include "fastFeedback.h"



#define FF_BASE_ADDRESS         0x14028000
#define FF_CONTROL_ADDRESS      0x1402A000
/* Offset from FF_BASE_ADDRESS of the status space registers.  This is
 * assumed to be on the same page. */
#define FF_STATUS_OFFSET        0x0C00


struct FF_CONFIG_SPACE          // At FF_BASE_ADDRESS (14028000)
{
    int BpmId;                          // BPMID
    int TimerFrameCountDown;            // FRAMELEN
    int PowerDown;                      // :ENABLE
    int LoopBack;                       // :LOOPBACK
};


struct FF_STATUS_SPACE          // At FF_BASE_ADDRESS + FF_STATUS_OFFSET
{
    int FirmwareVersion;                // VERSION
    int SystemStatus;                   //
    int LinkPartner[4];                 // :PARTNER
    int LinkUp;                         // :UP
    int TimeFrameCounter;               // TIMEFRAME
    int HardErrorCount[4];              // :HARD_ERR
    int SoftErrorCount[4];              // :SOFT_ERR
    int FrameErrorCount[4];             // :FRAME_ERR
    int ReceivedPacketCount[4];         // :RX_CNT
    int TransmittedPacketCount[4];      // :TX_CNT
    int ProcessTime;                    // PROCESS_TIME
    int BpmCount;                       // BPM_COUNT
};


struct FF_CONTROL_SPACE         // At FF_CONTROL_ADDRESS (1402A000)
{
    /* Fast Application Interface configuration control register.
     *
     *  bit 0 => Configuration data handshake.  Configuration data is read on
     *           the rising edge of this bit.
     *      1 => Data select.  0 => real position data, 1 => time frame
     *           counter data is sent instead of positions.
     *      2 => (Unused)
     *      3 => Communication controller enable/disable.  Controller is
     *           enabled when this bit is 1.
     *   4-31    (Unused)
     */
    int FaiConfiguration;
    /* Arm start of fast feedback processing by writing 1 to this register. */
    int ExternalTriggerStartMask;
    /* Write any value to this register to stop fast feedback clock. */
    int SoftwareStopControl;
    /* This register can be used to select the communication controller payload,
     * if the appropriate firmware is configured. */
    int FAPayload;
};



/* /dev/mem file handle used for access to FF control space. */
static int DevMem = -1;

/* Memory mapped page for access to configuration and status monitoring
 * space. */
static void * FF_AddressSpaceMem = (void *) -1;
/* Memory mapped page for access to control register. */
static void * FF_ControlSpaceMem = (void *) -1;

/* These pointers directly overlay the FF memory. */
static FF_CONFIG_SPACE * ConfigSpace;
static FF_STATUS_SPACE * StatusSpace;
static volatile FF_CONTROL_SPACE * ControlSpace;


/* Boolean values extracted from LinkUp field.  This array is updated once a
 * second. */
static bool TxLinkUp[4];
static bool RxLinkUp[4];

/* Mirrors of configuration values.  These values cannot be read and written
 * directly, and so are instead read and written here. */
static bool DataSourceSelect = false;
static bool GlobalEnable = true;
static bool LinkEnable[4] = { true, true, true, true };
static int LoopBack[4] = { 0, 0, 0, 0 };
static int XPayload = 14, YPayload = 15;



static bool MapFastFeedbackMemory()
{
    const int PageSize = getpagesize();         // 0x1000
    const int PageMask = PageSize - 1;          // 0x0FFF
    bool Ok =
        TEST_IO(DevMem = open("/dev/mem", O_RDWR | O_SYNC))  &&
        TEST_IO(FF_AddressSpaceMem = mmap(
            0, PageSize, PROT_READ | PROT_WRITE, MAP_SHARED,
            DevMem, FF_BASE_ADDRESS & ~PageMask))  &&
        TEST_IO(FF_ControlSpaceMem = mmap(
            0, PageSize, PROT_READ | PROT_WRITE, MAP_SHARED,
            DevMem, FF_CONTROL_ADDRESS & ~PageMask));
    if (Ok)
    {
        ConfigSpace = (FF_CONFIG_SPACE *) (void *)
            ((char *) FF_AddressSpaceMem + (FF_BASE_ADDRESS & PageMask));
        StatusSpace = (FF_STATUS_SPACE *) (void *)
            ((char *) ConfigSpace + FF_STATUS_OFFSET);
        ControlSpace = (volatile FF_CONTROL_SPACE *) (void *)
            ((char *) FF_ControlSpaceMem + (FF_CONTROL_ADDRESS & PageMask));
    }

    return Ok;
}


#define PUBLISH_BLOCK(record, Name, Fields) \
    for (int __i = 0; __i < 4; __i++) \
    { \
        char Link[32]; \
        sprintf(Link, "FF:LINK%d:", __i + 1); \
        Publish_##record(Concat(Link, Name), Fields[__i]); \
    }



/* This is called each time the status and monitor fields are about to be
 * read.  We simply update the up bits, as all other fields can be read
 * directly. */

static void ProcessRead()
{
    int UpMask = StatusSpace->LinkUp;
    for (int i = 0; i < 4; i ++)
    {
        RxLinkUp[i] = (UpMask & (1 << i)) != 0;
        TxLinkUp[i] = (UpMask & (1 << (i + 4))) != 0;
    }
}


/* Combines configured with dynamic values to generate a control register
 * value.  The Handshake bit is used to enable the reading of configuration
 * space values on its rising edge. */

inline static int ControlValue(bool Handshake)
{
    return
        (Handshake << 0) |
        (DataSourceSelect << 1) |
        (GlobalEnable << 3);
}


/* Called each time a configuration value has changed. */

static void ProcessWrite()
{
    /* For simplicity we assemble the PowerDown and LoopBack values each time
     * any field is written.  This should be harmless enough, as the only
     * side effects happen when the control register handshake is toggled
     * below. */
    ConfigSpace->PowerDown =
        (!LinkEnable[0] << 0) | (!LinkEnable[1] << 1) |
        (!LinkEnable[2] << 2) | (!LinkEnable[3] << 3);
    ConfigSpace->LoopBack =
        (LoopBack[0] << 0) | (LoopBack[1] << 2) |
        (LoopBack[2] << 4) | (LoopBack[3] << 6);

    /* Force the configuration values to be read by toggling the handshake
     * bit in the control register. */
    ControlSpace->FaiConfiguration = ControlValue(true);
    ControlSpace->FaiConfiguration = ControlValue(false);
}


/* When DataSourceSelect is changed, update control register *without*
 * performing configuration change handshake. */

static void WriteDataSourceSelect()
{
    ControlSpace->FaiConfiguration = ControlValue(false);
}


static void ProcessPayload()
{
    if (FAPayloadSelection)
        ControlSpace->FAPayload = ((YPayload & 0xF) << 4) | (XPayload & 0xF);
}


/* Routines called to stop and start fast feedback. */

static void StopFastFeedback()
{
    ControlSpace->ExternalTriggerStartMask = 0;
    ControlSpace->SoftwareStopControl = 0;
    ControlSpace->FaiConfiguration = 0;
}

static void StartFastFeedback()
{
    ControlSpace->ExternalTriggerStartMask = 0;
    ControlSpace->FaiConfiguration = ControlValue(false);
    ControlSpace->ExternalTriggerStartMask = 1;
}



bool InitialiseFastFeedback()
{
    /* If fast feedback was not detected, don't do anything at all. */
    if (!FastFeedbackFeature)
        return true;

    if (!MapFastFeedbackMemory())
        return false;

    /* Read only parameters. */
    Publish_longin("FF:VERSION", StatusSpace->FirmwareVersion);
    Publish_longin("FF:TIMEFRAME", StatusSpace->TimeFrameCounter);
    Publish_longin("FF:PROCESS_TIME", StatusSpace->ProcessTime);
    Publish_longin("FF:BPM_COUNT", StatusSpace->BpmCount);
    /* Channel specific read only parameters. */
    PUBLISH_BLOCK(longin, "PARTNER", StatusSpace->LinkPartner);
    PUBLISH_BLOCK(longin, "SOFT_ERR", StatusSpace->SoftErrorCount);
    PUBLISH_BLOCK(longin, "FRAME_ERR", StatusSpace->FrameErrorCount);
    PUBLISH_BLOCK(longin, "HARD_ERR", StatusSpace->HardErrorCount);
    PUBLISH_BLOCK(longin, "RX_CNT", StatusSpace->ReceivedPacketCount);
    PUBLISH_BLOCK(longin, "TX_CNT", StatusSpace->TransmittedPacketCount);
    PUBLISH_BLOCK(bi, "TX_UP", TxLinkUp);
    PUBLISH_BLOCK(bi, "RX_UP", RxLinkUp);

    /* The ProcessRead function updates the LinkUp array.  All the other
     * fields can be read directly by EPICS, as no special synchronisation or
     * other treatment is required. */
    PUBLISH_ACTION("FF:PROCESS", ProcessRead);

    /* Sensible defaults for frame length. */
    ConfigSpace->TimerFrameCountDown = 6000;

    PUBLISH_CONFIGURATION(longout, "FF:BPMID",
        ConfigSpace->BpmId, ProcessWrite);
    PUBLISH_CONFIGURATION(longout, "FF:FRAMELEN",
        ConfigSpace->TimerFrameCountDown, ProcessWrite);

    for (int i = 0; i < 4; i ++)
    {
        char Link[32];
        sprintf(Link, "FF:LINK%d:", i + 1);
        PUBLISH_FUNCTION_OUT(bo,   Concat(Link, "ENABLE"),
            LinkEnable[i], ProcessWrite);
        PUBLISH_FUNCTION_OUT(mbbo, Concat(Link, "LOOPBACK"),
            LoopBack[i], ProcessWrite);
    }
    PUBLISH_FUNCTION_OUT(bo, "FF:DATA_SELECT",
        DataSourceSelect, WriteDataSourceSelect);
    PUBLISH_CONFIGURATION(mbbo, "FF:XPAYLOAD", XPayload, ProcessPayload);
    PUBLISH_CONFIGURATION(mbbo, "FF:YPAYLOAD", YPayload, ProcessPayload);

    PUBLISH_ACTION("FF:STOP",  StopFastFeedback);
    PUBLISH_ACTION("FF:START", StartFastFeedback);

    /* Initialise the FPGA by writing the current configuration. */
    ProcessWrite();
    ProcessPayload();

    return true;
}
