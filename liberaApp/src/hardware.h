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


/* Exports for libera device driver interface routines. */



/*****************************************************************************/
/*                                                                           */
/*                             Data Structures                               */
/*                                                                           */
/*****************************************************************************/



/* A row of "data on demand" Libera data consists of eight integers
 * containing quadrature data (cos/sin pairs) for each of the four buttons
 * thus: 
 *      0, 1    A * (cos,sin)
 *      2, 3    B * (cos,sin)
 *      4, 5    C * (cos,sin)
 *      6, 7    D * (cos,sin)
 * Each (cos,sin) pair must be converted to magnitude before further
 * processing.
 *     This data can be read at machine revolution clock frequency, or
 * decimated to 1/64th. */
typedef int LIBERA_ROW[8];


/* The ADC data is read directly from the ADC converter at the sample rate of
 * 117MHz.  Each row consists of four 12-bit signed values (not sign extended
 * in the current release of the driver), one for each button.  ADC data is
 * always read in 1024 row segments. */
#define ADC_LENGTH      1024
typedef unsigned int PERMUTATION[4];
typedef short ADC_ROW[4];
struct ADC_DATA
{
    /* Button permutation corrresponding to current switch position. */
    PERMUTATION Permutation;
    /* Raw ADC data as read. */
    ADC_ROW Rows[ADC_LENGTH];
};


/* Slow acquisition data is read as four filtered button values. */
struct SA_DATA
{
    int A, B, C, D;
};


/* The attenuators for Libera are controlled as an array of 8 settings, two
 * per input channel, each of which can be any value in the range 0..31 --
 * this corresponds to precisely this attenuation in dB. */
typedef unsigned int ATTENUATORS[8];




/*****************************************************************************/
/*                                                                           */
/*                       Hardware Interface Routines                         */
/*                                                                           */
/*****************************************************************************/

/* To be called once on startup to initialise connection to Libera device.
 * If this routine fails (and returns false) then no further operations can
 * be done and system startup should fail. */
bool InitialiseHardware();

/* To be called on shutdown to release all connections to Libera. */
void TerminateHardware();



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                   Reading waveform data from the FPGA.                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Reads a waveform of the given length (number of rows) using the given
 * decimation into the given block of data.  Returns the number of rows
 * actually read.
 *     At present only decimation values of 1 or 64 are suppported. */
size_t ReadWaveform(int Decimation, size_t WaveformLength, LIBERA_ROW * Data);

/* Reads the postmortem buffer. */
size_t ReadPostmortem(size_t WaveformLength, LIBERA_ROW * Data);

/* Reads a full 1024 point ADC waveform. */
bool ReadAdcWaveform(ADC_DATA &Data);

/* Reads a slow acquisition update. */
bool ReadSlowAcquisition(SA_DATA &Data);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      Miscellaneous Support Routines.                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Routines to read and write attenuator settings. */
bool ReadAttenuators(ATTENUATORS attenuators);
bool WriteAttenuators(const ATTENUATORS attenuators);

/* Read and write switch settings. */
bool ReadSwitches(int &switches);
bool WriteSwitches(int switches);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Direct Event Connection                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifdef _CSPI_H
/* The following event types can be returned by the event mechanism.  These
 * are a digest of corresponding events in libera_event_id_t, or equivalently,
 * CSPI_EVENTMASK in cspi.h.  We conditionally include these definitions here
 * to avoid including cspi.h outside the core hardware.cpp file. */
enum HARDWARE_EVENT_ID
{
    /* Something has changed in the configuration.  It may be useful to
     * monitor this to track values of switches and attenuators. */
    HARDWARE_EVENT_CFG       = CSPI_EVENT_CFG,
    /* Slow acquisition sample available.  This should tick at 10Hz.
     * Unfortunately it appears that, at least with the current version of
     * the driver, this event never arrives! */
    HARDWARE_EVENT_SA        = CSPI_EVENT_SA,
    /* Postmortem event.  A fixed 16K sample buffer is available. */
    HARDWARE_EVENT_PM        = CSPI_EVENT_PM,
    /* Normal triggered data is available. */
    HARDWARE_EVENT_TRIGGET   = CSPI_EVENT_TRIGGET
};


/* Reads one event from the event queue and decodes it into an event id and
 * its associated parameter information.  The caller should empty the queue
 * by calling this routine until it returns false before processing any
 * events. */
bool ReadOneEvent(HARDWARE_EVENT_ID &Id, int &Param);

/* This value can be used in a select() call to discover when ReadOneEvent()
 * should be called. */
int EventSelector();
#endif



/*****************************************************************************/
/*                                                                           */
/*                          Generic Helper Macros                            */
/*                                                                           */
/*****************************************************************************/

/* Helper macro for OS I/O commands which return -1 on error and set errno.
 * Arguments are:
 *      var     Variable to receive result of command
 *      error   Error message to print on error
 *      command I/O command to call
 *      args    Arguments to I/O command
 * Return true on success, prints an error message and returns false if the
 * I/O command fails. */
#define TEST_IO(var, error, command, args...) \
    ( var = command(args), \
      var == -1 ? perror(error), false : true )

/* Much the same as for TEST_IO, but for I/O commands which return 0 on
 * success and an ignorable non-zero code on failure.  No variable needs to be
 * specified. */
#define TEST_RC(error, command, args...) \
    ( command(args) != 0 ? perror(error), false : true )

/* An even more simplified version of TEST_RC, where the error string is
 * simply the function name. */
#define TEST_(command, args...)  TEST_RC(#command, command, args)
