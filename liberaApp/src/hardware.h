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

/* The data structure declarations here are essentially duplicates of
 * corresponding definitions in libera.h.  This is done to avoid having to
 * #include libera.h throughout this code.
 *
 * The correspondence between definitions in this file and CSPI/Libera
 * definitions is as follows:
 *
 * LIBERA_TIMESTAMP     Near copy of libera_timestamp_t, libera.h
 * LIBERA_ROW           Derived from libera_dd_atomic_t, libera.h
 *                      and CSPI_DD_ATOM, cspi.h
 * LIBERA_DATA          Inferred from cspi_read_ex, cspi.c
 * ATTENUATORS          Inferred from set_envparam, get_envparam, cspi.c
 * LIBERA_EVENT_ID      Renamed copy of libera_nm_t, libera.h
 *
 * Note that only two definitions are direct copies (with renaming to avoid
 * name clashes inside hardware.cpp), as CSPI provide a certain degree of
 * abstraction which has to be unwound here. */



struct LIBERA_TIMESTAMP
{
    struct timespec st;
    unsigned long long int mt;
};

/* The data returned by Libera comes back as an array of rows, 8 doublewords
 * per row, preceded by a timestamp. */
typedef int LIBERA_ROW[8];
struct LIBERA_DATA
{
    LIBERA_TIMESTAMP Timestamp;
    LIBERA_ROW Rows[0];
};

#define LIBERA_DATA_SIZE(RowCount) \
    (sizeof(LIBERA_DATA) + (RowCount) * sizeof(LIBERA_ROW))


#define ADC_LENGTH      1024
typedef short ADC_ROW[4];
struct ADC_DATA
{
    ADC_ROW Rows[ADC_LENGTH];
};


struct SA_DATA
{
    // Amplitudes
    int Va, Vb, Vc, Vd;
    // Sum Va + Vb + Vc + Vd
    int Sum;
    // Quadropole signal
    int Q;
    // Horizontal beam position
    int X;
    // Vertical beam position
    int Y;
    // Horiz. and vert. correction factors from the FA Application
    int Cx, Cy;
    // 6 values reserved for future use
    int reserved[6];
};


/* The attenuators for Libera are controlled as an array of 8 settings, two
 * per input channel, each of which can be any value in the range 0..31 --
 * this corresponds to precisely this attenuation in dB. */
typedef unsigned int ATTENUATORS[8];


/* The following event types can be returned by the event mechanism.   These
 * are a digest of corresponding events in libera_event_id_t. */
enum HARDWARE_EVENT_ID
{
    /* Internal device driver queue has overflowed.  This means that events
     * have been lost, but frankly there's not a lot we can do about it. */
    HARDWARE_EVENT_OVERFLOW  = 0,
    /* Something has changed in the configuration.  It may be useful to
     * monitor this to track values of switches and attenuators. */
    HARDWARE_EVENT_CFG       = 1,
    /* Slow acquisition sample available.  This should tick at 10Hz. */
    HARDWARE_EVENT_SA        = 2,
    /* Interlock event.  Machine interlock status change(?) */
    HARDWARE_EVENT_INTERLOCK = 3,
    /* Post mortem trigger.  Everything has come to a halt, now it's time to
     * see why.  Does everything need to be reenabled after this??? */
    HARDWARE_EVENT_PM        = 4,
    /* Fast acquisition event.  What causes this, do we need it??? */
    HARDWARE_EVENT_FA        = 5,
    /* This seems to be the normal external trigger event. */
    HARDWARE_EVENT_TRIGGET   = 6,
    /* "SET Trigger trigger" -- what on earth is this??? */
    HARDWARE_EVENT_TRIGSET   = 7,
    /* And what on earth is this for? */
    HARDWARE_EVENT_USER      = 31
};



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
size_t ReadWaveform(int Decimation, size_t WaveformLength, LIBERA_DATA & Data);

/* Reads a full 1024 point ADC waveform. */
bool ReadAdcWaveform(ADC_DATA &Data);

/* Reads a slow acquisition update. */
bool ReadSlowAcquisition(SA_DATA &Data);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      Miscellaneous Support Routines.                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Routines to read and write attenuator settings. */
bool ReadAttenuators(ATTENUATORS attenuators);
bool WriteAttenuators(ATTENUATORS attenuators);

/* Read and write switch settings. */
bool ReadSwitches(int &switches);
bool WriteSwitches(int switches);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Direct Event Connection                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Reads one event from the event queue and decodes it into an event id and
 * its associated parameter information.  The caller should empty the queue
 * by calling this routine until it returns false before processing any
 * events. */
bool ReadOneEvent(HARDWARE_EVENT_ID &Id, int &Param);

/* Enables delivery of events.  Until this is called ReadOneEvent() will
 * always return false. */
bool OpenEventStream();

/* This value can be used in a select() call to discover when ReadOneEvent()
 * should be called. */
int EventSelector();



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



/* The following macro counts the number of leading zeros in the argument in
 * and writes the result into clz. */
#define CLZ(in, clz) __asm__("clz %0,%1" : "=r"(clz) : "r"(in))
