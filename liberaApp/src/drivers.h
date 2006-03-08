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


/* Common interface to epics records. */

#include <alarm.h>
struct dbCommon;
struct waveformRecord;
struct subArrayRecord;


/* This class is instanced by driver.cpp to provide I/O Intr callback
 * functionality. */
class I_INTR
{
public:
    /* This routine can be called asynchronously (for example, in response to
     * a signal) at any time after EnableIoIntr() has been called to trigger
     * subsequent processing of the associated record.
     *    True is returned iff the signal has actually been passed through to
     * EPICS.  A return of false indicates that either EPICS is not
     * configured to use interrupts on this record, or that this record has
     * not yet bee initialised. */
    virtual bool IoIntr() = 0;
};


/* This class provides for support for I/O Intr scanning and notification,
 * and provides a default implementation without I/O Intr.  All record
 * interfaces will subclass this.
 *    Think of this as a pure interface with some simple handy default
 * implementations built in for convenience. */
class I_RECORD
{
public:
    /* This method will be called to ask whether I/O Intr support should be
     * provided.  If so then true should be returned and the I_INTR interface
     * pointer retained and called as appropriate.
     *    The default implementation does not support I/O Intr scanning. */
    virtual bool EnableIoIntr(I_INTR & Intr) { return false; }

    /* This method will be called after the record has been found by the
     * EPICS driver to allow any record specific customisation to be done.
     * If the implementation then discovers that other settings on the record
     * are inappropriate it can return false to reject the binding.
     *    The default implementation does nothing and returns success. */
    virtual bool BindRecord(dbCommon * pr) { return true; }

    /* This method will be called immediately after the record's read or
     * write method (as defined below for processing).  An alarm state can be
     * returned if required.
     *    The default implementation returns no alarm. */
    virtual epicsAlarmSeverity AlarmStatus() { return epicsSevNone; }
};


/*****************************************************************************/
/*                          Record Type Interfaces */
/*****************************************************************************/
/* The following interfaces define the functionality that should be provided
 * to implement each class of record. */


/* Helper macros for determining the underlying type for each supported
 * record type. */
#define TYPEOF(record)  TYPEOF_##record

#define TYPEOF_longin   int
#define TYPEOF_longout  int
#define TYPEOF_ai       double
#define TYPEOF_ao       double
#define TYPEOF_bi       bool
#define TYPEOF_bo       bool


/* Both read and write methods return true if the operation was successful,
 * false if it failed.  Failure of read is unlikely, but supported! */

template<class T> class I_READER : public I_RECORD
{
public:
    virtual bool read(T &) = 0;
};

template<class T> class I_WRITER : public I_RECORD
{
public:
    virtual bool init(T&) = 0;
    virtual bool write(T) = 0;
};

#define I_IN_VARIABLE(record)    typedef I_READER<TYPEOF(record)> I_##record
#define I_OUT_VARIABLE(record)   typedef I_WRITER<TYPEOF(record)> I_##record


I_IN_VARIABLE(longin);             // I_longin
I_OUT_VARIABLE(longout);           // I_longout
I_IN_VARIABLE(ai);                 // I_ai
I_OUT_VARIABLE(ao);                // I_ao
I_IN_VARIABLE(bi);                 // I_bi
I_OUT_VARIABLE(bo);                // I_bo


class I_waveform : public I_RECORD
{
public:
    /* Read the waveform.  
     *     The number of waveform rows actually read is returned: if reading
     * fails then 0 is returned. */
    virtual size_t read(void *array, size_t length) = 0;
};


/* This is a helper class which provides a simple BindRecord function which
 * implements the routine step of validating the array data type. */
class I_WAVEFORM : public I_waveform
{
public:
    /* Instances of this class will automatically validate their waveform
     * types and refuse to accept EPICS connections with an unexpected record
     * type. */
    I_WAVEFORM(epicsEnum16 Type);
    
    bool BindRecord(dbCommon * pr);

private:
    const epicsEnum16 Type;
};
