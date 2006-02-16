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
#include "ebpp.h"
#include "hardware.h"
#include "support.h"



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
int libera_event = -1;
/* Libera ADC device, used to read ADC rate waveforms. */
int libera_adc = -1;
/* Libera SA (Slow Acquisition) device, used to receive 10Hz updates. */
int libera_sa = -1;


/* We record the currently set decimation setting here.  This allows us to
 * avoid writing a new decimation setting if it is unchanged (whether this
 * saves anything is questionable, though). */
int CurrentDecimation;




/*****************************************************************************/
/*                                                                           */
/*                      Miscellaneous Support Routines.                      */
/*                                                                           */
/*****************************************************************************/


/* Libera driver configuration parameters are read and written in a slightly
 * roundabout way: a common request structure (a pair of ints) consisting of
 * a request code and the value to be read or written.  Request codes are
 * shared between reads or writes, and the actual reading and writing
 * operations are done through dedicated ioctl commands on the .cfg device. */

bool ReadCfgValue(unsigned int RequestCode, unsigned int &Value)
{
    libera_cfg_request_t Request = { RequestCode, 0 };
    bool Ok = TEST_RC("Unable to read configuration value",
        ioctl, libera_cfg, LIBERA_IOC_GET_CFG, &Request);
    Value = Request.val;
    return Ok;
}

bool WriteCfgValue(unsigned int RequestCode, unsigned int Value)
{
    libera_cfg_request_t Request = { RequestCode, Value };
    return TEST_RC("Unable to write configuration value",
        ioctl, libera_cfg, LIBERA_IOC_SET_CFG, &Request);
}



/* We think of the attenuators in the ATTENUATORS array as setting values for
 * the first and second stages of channels 1 to 4, in the order
 *      ch1 1st, ch1 2nd, ch2 1st, ..., ch4 2nd . */

const unsigned int AttenCfg[] = {
    LIBERA_CFG_ATTN0, LIBERA_CFG_ATTN1, LIBERA_CFG_ATTN2, LIBERA_CFG_ATTN3, 
    LIBERA_CFG_ATTN4, LIBERA_CFG_ATTN5, LIBERA_CFG_ATTN6, LIBERA_CFG_ATTN7
};

bool ReadAttenuators(ATTENUATORS Attenuators)
{
    bool Ok = true;
    for (int i = 0; i < 8 && Ok; i ++)
        Ok = ReadCfgValue(AttenCfg[i], Attenuators[i]);
    return Ok;
}


bool WriteAttenuators(ATTENUATORS Attenuators)
{
    bool Ok = true;
    for (int i = 0; i < 8 && Ok; i ++)
    {
        /* Check that the values are in range. */
        if (Attenuators[i] >= 32)
        {
            printf("Attenuator %d setting %d out of range\n",
                i, Attenuators[i]);
            Ok = false;
        }
    }
    for (int i = 0; i < 8 && Ok; i ++)
        Ok = WriteCfgValue(AttenCfg[i], Attenuators[i]);
    return Ok;
}

bool ReadSwitches(int &Switches)
{
    return ReadCfgValue(LIBERA_CFG_SWITCH, *(unsigned *)&Switches);
    // Um.  Close your eyes for the ugly cast.
}


bool WriteSwitches(int Switches)
{
    if (0 <= Switches  &&  Switches < 16)
        return WriteCfgValue(LIBERA_CFG_SWITCH, Switches);
    else
    {
        printf("Switch value %d out of range\n", Switches);
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




size_t ReadWaveform(int Decimation, size_t WaveformLength, LIBERA_ROW * Data)
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

    /* Read the data in two parts.  The waveform proper is read directly into
     * the data part of the waveform; we then read the timestamp as a
     * separate ioctl operation. */
    const ssize_t DataSize = WaveformLength * sizeof(LIBERA_ROW);
    ssize_t bytes_read;
    TEST_IO(bytes_read, "Problem reading waveform",
        read, libera_dd, Data, DataSize);
    if (bytes_read != DataSize)
        printf("Couldn't read entire block: read %d\n", bytes_read);

    /* Convert bytes read into a number of rows. */
    if (bytes_read <= 0)
        return 0;
    else
        return bytes_read / sizeof(LIBERA_ROW);
}


bool ReadAdcWaveform(ADC_DATA &Data)
{
    const ssize_t DataSize = ADC_LENGTH * sizeof(ADC_ROW);
    ssize_t bytes_read;
    return TEST_IO(bytes_read, "Problem reading ADC waveform",
        read, libera_adc, &Data.Rows, DataSize)  &&
        bytes_read == DataSize;
}


bool ReadSlowAcquisition(SA_DATA &SaData)
{
    libera_atom_sa_t Data;
    const ssize_t DataSize = sizeof(libera_atom_sa_t);
    ssize_t bytes_read;
    TEST_IO(bytes_read, "Problem reading SA data",
        read, libera_sa, &Data, DataSize);
    if (bytes_read == DataSize)
    {
        SaData.A = Data.Va;
        SaData.B = Data.Vb;
        SaData.C = Data.Vc;
        SaData.D = Data.Vd;  
        return true;
    }
    else
    {
        printf("Couldn't read entire block: read %d\n", bytes_read);
        return false;
    }
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

bool ReadOneEvent(HARDWARE_EVENT_ID &Id, int &Param)
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
            /* Good.  Return the event.
             * In the current instantiation of the Libera driver we receive
             * an event as a bit.  Here we convert that bit back into an
             * event id.  If we receive more that one event in this way it
             * will be lost! */
            /* Count the leading zeros and convert the result into a count of
             * trailing zero. */
            Id = (HARDWARE_EVENT_ID) (31 - CLZ(event.id));
            Param = event.param;
            if (event.id != 1 << Id)
                printf("Unexpected event id %08x\n", event.id);
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

//bool OpenEventStream(size_t EventMask)  !!! Come back to this
bool OpenEventStream()
{
    /* Not a great deal of point in asking for the SA event, as it never
     * arrives: a place-holder in case it does one day. */
    size_t EventMask = LIBERA_EVENT_TRIGGET | LIBERA_EVENT_SA;
    /* Enable delivery of the events we're really interested in. */
    return TEST_RC("Unable to enable events",
        ioctl, libera_event, LIBERA_EVENT_SET_MASK, &EventMask);
}


/* Because the event consumer needs to use select() to wait for event
 * delivery we need to break encapsulation here and expose the actual handle
 * used to receive events. */

int EventSelector()
{
    return libera_event;
}


/* Called on termination to cancel event delivery. */

static void CloseEventStream()
{
    size_t EventMask = 0;
    TEST_RC("Unable to disable events",
        ioctl, libera_event, LIBERA_EVENT_SET_MASK, &EventMask);
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
        TEST_IO(libera_adc, "Unable to open libera.adc device",
            open, "/dev/libera.adc", O_RDONLY)  &&
        TEST_IO(libera_sa, "Unable to open libera.sa device",
            open, "/dev/libera.sa", O_RDONLY)  &&
        TEST_IO(libera_event, "Unable to open event fifo",
            open, "/dev/libera.event", O_RDWR | O_NONBLOCK)  &&
        /* Start with the current settings of Libera read.  (This is not
         * strictly necessary, but harmless.) */
        TEST_RC("Unable to read decimation",
            ioctl, libera_dd, LIBERA_IOC_GET_DEC, &CurrentDecimation)  &&
        /* Initialise the hardware and multiplexor switches into a good known
         * state. */
        WriteAttenuators(Attenuators)  &&
        WriteSwitches(0);
}


/* To be called on shutdown to release all connections to Libera. */
void TerminateHardware()
{
    if (libera_dd != -1)   close(libera_dd);
    if (libera_adc != -1)  close(libera_adc);
    if (libera_cfg != -1)  close(libera_cfg);
    if (libera_sa != -1)   close(libera_sa);
    if (libera_event != -1)
    {
        CloseEventStream();
        close(libera_event);
    }
}
