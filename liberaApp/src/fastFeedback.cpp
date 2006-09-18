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


/* Support for configuration access to fast feedback control registers.
 * Diamond Light Source specific code. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>


#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "thread.h"
#include "trigger.h"
#include "hardware.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"

#include "fastFeedback.h"



#define FF_BASE_ADDRESS         0x1402A000
#define FF_CONTROL_ADDRESS      0x14029FFC

#define PAGE_SIZE               0x1000
#define PAGE_MASK               (-PAGE_SIZE)


struct FF_CONFIG_SPACE
{
    int BpmId;                          // BPMID
    int TimerFrameCountDown;            // FRAMELEN
    int PowerDown;                      // :ENABLE
    int LoopBack;                       // :LOOPBACK
    int ClearDelay;                     // CLEAR_DELAY
} __attribute__((packed));


struct FF_STATUS_SPACE
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
    short ProcessTimeMin;               //
    short ProcessTimeMax;               //
    int ProcessTime;                    //
    int BpmCount;                       //
    int TestErrorStatus;                //
} __attribute__((packed));



/* /dev/mem file handle used for access to FF control space. */
static int DevMem = -1;

/* Memory mapped page for access to configuration and status monitoring
 * space. */
static void * FF_AddressSpace = (void *) -1;
/* Memory mapped page for access to control register. */
static void * FF_ControlSpace = (void *) -1;

/* These three pointers directly overlay the FF memory. */
static FF_CONFIG_SPACE * ConfigSpace;
static FF_STATUS_SPACE * StatusSpace;
static volatile int * ControlRegister;


/* Boolean values extracted from LinkUp field.  This array is updated once a
 * second. */
static bool LinkUp[4];

/* Mirrors of configuration values.  These values cannot be read and written
 * directly, and so are instead read and written here. */
static bool SendTimeFrames = false;
static bool GlobalEnable = true;
static bool LinkEnable[4] = { true, true, true, true };
static int LoopBack[4] = { 0, 0, 0, 0 };

/* This boolean value is written but ignored: used for actions. */
static bool Ignored = false;


static bool MapFastFeedbackMemory()
{
    bool Ok = 
        TEST_IO(DevMem, "Opening /dev/mem",
            open, "/dev/mem", O_RDWR | O_SYNC)  &&
        TEST_IO(FF_AddressSpace, "Mapping memory",
            mmap, 0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            DevMem, FF_BASE_ADDRESS)  &&
        TEST_IO(FF_ControlSpace, "Mapping memory",
            mmap, 0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
            DevMem, FF_CONTROL_ADDRESS & PAGE_MASK);
    if (Ok)
    {
        ConfigSpace = (FF_CONFIG_SPACE *) FF_AddressSpace;
        StatusSpace = (FF_STATUS_SPACE *) ((char *) FF_AddressSpace + 0x200);
        ControlRegister = (volatile int *)
            (((char *) FF_ControlSpace +
                (FF_CONTROL_ADDRESS & (PAGE_SIZE-1))));
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
        LinkUp[i] = (UpMask & (1 << i)) != 0;
}


/* Combines configured with dynamic values to generate a control register
 * value.  The Handshake bit is used to enable the reading of configuration
 * space values on its rising edge. */

inline static int ControlValue(bool Handshake, bool SendBpmId)
{
    return
        (Handshake << 0) | 
        (SendTimeFrames << 1) | 
        (SendBpmId << 2) | 
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
    *ControlRegister = ControlValue(true, false);
    *ControlRegister = ControlValue(false, false);
}



static void ProcessSendId()
{
    /* Request sending of BPM id by toggling the send id bit. */
    *ControlRegister = ControlValue(false, true);
    *ControlRegister = ControlValue(false, false);
}




bool InitialiseFastFeedback()
{
    if (!MapFastFeedbackMemory())
        return false;

    /* Read only parameters. */
    Publish_longin("FF:VERSION", StatusSpace->FirmwareVersion);
    Publish_longin("FF:TIMEFRAME", StatusSpace->TimeFrameCounter);
    /* Channel specific read only parameters. */
    PUBLISH_BLOCK(longin, "PARTNER", StatusSpace->LinkPartner);
    PUBLISH_BLOCK(longin, "SOFT_ERR", StatusSpace->SoftErrorCount);
    PUBLISH_BLOCK(longin, "FRAME_ERR", StatusSpace->FrameErrorCount);
    PUBLISH_BLOCK(longin, "HARD_ERR", StatusSpace->HardErrorCount);
    PUBLISH_BLOCK(longin, "RX_CNT", StatusSpace->ReceivedPacketCount);
    PUBLISH_BLOCK(longin, "TX_CNT", StatusSpace->TransmittedPacketCount);
    PUBLISH_BLOCK(bi, "UP", LinkUp);

    /* The ProcessRead function updates the LinkUp array.  All the other
     * fields can be read directly by EPICS, as no special synchronisation or
     * other treatment is required. */
    PUBLISH_FUNCTION_OUT(bo, "FF:PROCESS", Ignored, ProcessRead);

    
    PUBLISH_CONFIGURATION("FF:BPMID", longout,
        ConfigSpace->BpmId, ProcessWrite);
    PUBLISH_CONFIGURATION("FF:FRAMELEN", longout,
        ConfigSpace->TimerFrameCountDown, ProcessWrite);
    PUBLISH_CONFIGURATION("FF:CLEAR_DELAY", longout,
        ConfigSpace->ClearDelay, ProcessWrite);

    for (int i = 0; i < 4; i ++)
    {
        char Link[32]; 
        sprintf(Link, "FF:LINK%d:", i + 1); 
        PUBLISH_FUNCTION_OUT(bo,   Concat(Link, "ENABLE"),
            LinkEnable[i], ProcessWrite);
        PUBLISH_FUNCTION_OUT(mbbo, Concat(Link, "LOOPBACK"),
            LoopBack[i], ProcessWrite);
    }
    PUBLISH_FUNCTION_OUT(bo, "FF:DATA_SELECT", SendTimeFrames, ProcessWrite);
    PUBLISH_FUNCTION_OUT(bo, "FF:ENABLE", GlobalEnable, ProcessWrite);
    
    PUBLISH_FUNCTION_OUT(bo, "FF:SEND_ID", Ignored, ProcessSendId);

    
    /* Initialise the FPGA by writing the current configuration. */
    ProcessWrite();
    
    return true;
}



void TerminateFastFeedback()
{
    if (ConfigSpace != NULL)
        TEST_(munmap, FF_AddressSpace, PAGE_SIZE);
    if (StatusSpace != NULL)
        TEST_(munmap, FF_ControlSpace, PAGE_SIZE);
    if (DevMem != -1)
        close(DevMem);
}