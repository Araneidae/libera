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


/* Core device driver interface routines for access to Libera device. */


#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/unistd.h>

#include "libera.h"
#include "hardware.h"




/* ReadWaveform is defined to use _llseek which we implement here as a system
 * call.  Hopefully we can get rid of this by moving to lseek: we don't really
 * need long long offsets! */
static int _llseek(
    unsigned int fd,
    unsigned long offset_high,  unsigned long offset_low,
    long long * result, unsigned int origin)
{
    return syscall(__NR__llseek, fd, offset_high, offset_low, result, origin);
}




/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/

/* The following file handles are held open for the duration of the operation
 * of this driver. */

/* Libera configuration device, used to interrogate the switches, attenuators
 * and other general configuration settings. */
int libera_cfg = -1;
/* Libera DD (Direct Data) device, used to read waveforms. */
int libera_dd = -1;
/* Libera low level device, used to received event notification. */
int libera_low = -1;


/* We record the currently set decimation setting here.  This allows us to
 * avoid writing a new decimation setting if it is unchanged (whether this
 * saves anything is questionable, though). */
int CurrentDecimation;




/*****************************************************************************/
/*                                                                           */
/*                      Miscellaneous Support Routines.                      */
/*                                                                           */
/*****************************************************************************/


/* We think of the attenuators in the ATTENUATORS array as setting values for
 * the first and second stages of channels 1 to 4, in the order
 *      ch1 1st, ch1 2nd, ch2 1st, ..., ch4 2nd .
 * Unfortunately, the ordering actually supported by the device driver is a
 * little different.  This array is used to map channels in this order into
 * driver channels.
 *                       0   1   2   3   4   5   6   7
 *                      c11 c12 c21 c22 c31 c32 c41 c42 */
const int AttenMap[] = { 6,  7,  2,  3,  0,  1,  4,  5 };


bool ReadAttenuators(ATTENUATORS attenuators)
{
    ATTENUATORS remapped;
    if (TEST_RC("Unable to read attenuators",
        ioctl, libera_cfg, LIBERA_IOC_GET_ATTN, remapped))
    {
        for (int i = 0; i < 8; i ++)
             attenuators[i] = remapped[AttenMap[i]];
        return true;
    }
    else
        return false;
}


bool WriteAttenuators(ATTENUATORS attenuators)
{
    ATTENUATORS remapped;
    for (int i = 0; i < 8; i ++)
    {
        /* Simultaneously remap the attenuator settings and check that the
         * values are in range. */
        unsigned char a = attenuators[i];
        if (a < 32)
            remapped[AttenMap[i]] = a;
        else
        {
            printf("Attenuator %d setting %d out of range\n", i, a);
            return false;
        }
    }
    return TEST_RC("Unable to write attenuators",
        ioctl, libera_cfg, LIBERA_IOC_SET_ATTN, remapped);
}

bool ReadSwitches(int &switches)
{
    return TEST_RC("Unable to read switch settings",
        ioctl, libera_cfg, LIBERA_IOC_GET_SWITCH, &switches);
}


bool WriteSwitches(int switches)
{
    if (0 <= switches  &&  switches < 16)
        return TEST_RC("Unable to set switches",
            ioctl, libera_cfg, LIBERA_IOC_SET_SWITCH, &switches);
    else
    {
        printf("Switch value %d out of range\n", switches);
        return false;
    }
}



/* Used to control decimation settings on the DD device. */

static bool SetDecimation(int Decimation)
{
    /* Check the decimation request is valid. */
    if (Decimation != 1  &&  Decimation != 64)
    {
        printf("Invalid decimation %d\n", Decimation);
        return false;
    }

    /* Don't bother if that setting is already in place. */
    if (Decimation == CurrentDecimation)
        return true;

    /* Tell the hardware. */
    bool Ok = TEST_RC("Unable to write decimation",
         ioctl, libera_dd, LIBERA_IOC_SET_DEC, &Decimation);
    if (Ok)
        CurrentDecimation = Decimation;
    return Ok;
}



/*****************************************************************************/
/*                                                                           */
/*                   Reading waveform data from the FPGA.                    */
/*                                                                           */
/*****************************************************************************/




size_t ReadWaveform(int Decimation, size_t WaveformLength, LIBERA_DATA & Data)
{
    /* First set the requested decimation and then seek directly to the
     * trigger.  We'll read all data following directly after the trigger
     * point.
     *     The trigger point is defined as SEEK_END. */
    long long unused;
    bool Ok =
        SetDecimation(Decimation)  &&
        TEST_RC("Unable to seek to trigger point",
            _llseek, libera_dd, 0, 0, &unused, SEEK_END);
    if (!Ok)
        return 0;

    /* Try and grab the requested amount of data.  Here Length is the number
     * of waveform points, but we're really reading into a LIBERA_DATA
     * structure consisting of a leading timestamp, and each waveform point
     * is a row.  So here we calculate the desired data size. */
    const ssize_t DataSize = LIBERA_DATA_SIZE(WaveformLength);
    ssize_t bytes_read;
    TEST_IO(bytes_read, "Problem reading waveform",
        read, libera_dd, &Data, DataSize);
    if (bytes_read != DataSize)
        printf("Couldn't read entire block: read %d\n", bytes_read);
    /* Convert bytes read into a number of rows. */
    bytes_read -= sizeof(LIBERA_DATA);
    if (bytes_read <= 0)
        return 0;
    else
        return bytes_read / sizeof(LIBERA_ROW);
}



/*****************************************************************************/
/*                                                                           */
/*                        Direct Event Connection                            */
/*                                                                           */
/*****************************************************************************/



/* Reads one event from the event queue and decode it into an event id and
 * its associated parameter information.  The caller should empty the queue
 * by calling this routine until it returns false before processing any
 * events. */

bool ReadOneEvent(LIBERA_EVENT_ID &Id, int &Param)
{
    libera_event_t event;
    ssize_t bytes_read = read(libera_low, &event, sizeof(event));
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
            Id = (LIBERA_EVENT_ID) event.msg_id;
            Param = event.msg_param;
            return true;
        }
    }
    else
    {
        perror("Error reading event queue");
        return false;
    }
}


/* To be called on initialisation to enable delivery of events. */

bool OpenEventStream()
{
    return TEST_RC("Unable to enable events",
        ioctl, libera_low, LIBERA_LOW_SET_EVENT, 1);
}


/* Because the event consumer needs to use select() to wait for event
 * delivery we need to break encapsulation here and expose the actual handle
 * used to receive events. */

int EventSelector()
{
    return libera_low;
}


/* Called on termination to cancel event delivery. */

static void CloseEventStream()
{
    TEST_RC("Unable to disable events",
        ioctl, libera_low, LIBERA_LOW_SET_EVENT, 0);
}



/*****************************************************************************/
/*                                                                           */
/*                      Initialisation and Shutdown                          */
/*                                                                           */
/*****************************************************************************/


bool InitialiseHardware()
{
    /* A default attenuator setting of 20/20 (40 dB total) ensures a safe
     * starting point on initialisation. */
    ATTENUATORS Attenuators;
    for (int i = 0; i < 8; i ++)  Attenuators[i] = 20;
    
    return
        /* Permanently open the hardware devices used to communicate with
         * Libera. */
        TEST_IO(libera_cfg, "Unable to open libera.cfg device",
            open, "/dev/libera.cfg", O_RDWR)  &&
        TEST_IO(libera_dd, "Unable to open libera.dd device",
            open, "/dev/libera.dd", O_RDONLY)  &&
        TEST_IO(libera_low, "Unable to open event fifo",
            open, "/dev/libera.low", O_RDWR | O_NONBLOCK)  &&
        /* Start with the current settings of Libera read.  (This is not
         * strictly necessary, but harmless.) */
        TEST_RC("Unable to read decimation",
            ioctl, libera_dd, LIBERA_IOC_GET_DEC, &CurrentDecimation)  &&
        /* Initialise the hardware and multiplexor switches into a good known
         * state. */
        WriteAttenuators(Attenuators)  &&
        WriteSwitches(3);
}


/* To be called on shutdown to release all connections to Libera. */
void TerminateHardware()
{
    if (libera_dd != -1)   close(libera_dd);
    if (libera_cfg != -1)  close(libera_cfg);
    if (libera_low != -1)
    {
        CloseEventStream();
        close(libera_low);
    }
}
