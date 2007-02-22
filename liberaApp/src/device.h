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
     * not yet been initialised. */
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

    /* This method is also called immediately after other process, and can be
     * overridden to define a custom timestamp.  To ensure that this
     * timestamp isn't subsequently overwritten by record support, the record
     * definition should set TSE=-2 (and ensure TSEL is not set).
     *    Note that the timestamp here should be in Unix time, not EPICS
     * time: the appropriate correction is done in device.cpp. */
    virtual bool GetTimestamp(struct timespec &time) { return false; }
};


/*****************************************************************************/
/*                          Record Type Interfaces */
/*****************************************************************************/
/* The following interfaces define the functionality that should be provided
 * to implement each class of record. */


/* Both read and write methods return true if the operation was successful,
 * false if it failed.  Failure of read is unlikely, but supported! */

template<class T> class I_READER : public I_RECORD
{
public:
    /* Reads the current value. */
    virtual bool read(T &) = 0;
};

template<class T> class I_WRITER : public I_RECORD
{
public:
    /* Reads the initial underlying value.  This is useful for persistent
     * values, where this picks up the old stored state. */
    virtual bool init(T&) = 0;
    /* Writes a new value. */
    virtual bool write(T) = 0;
};


/* Epics strings are rather limited: a massive 39 characters are available! */
typedef char EPICS_STRING[40];

/* The following indulgence is an extravagantly efficient routine for copying
 * epics strings: I believe this cannot be done more efficiently!  Of course,
 * this is an exercise in futility, as epics strings have a negligible role
 * in this driver, but it's a nice bit of assembler. */
inline void CopyEpicsString(const EPICS_STRING in, EPICS_STRING &out)
{
    /* The choice of working registers here is a slightly delicate matter.  If
     * we just let the compiler allocate them for us then we get warning
     * messages from the assembler because the registers aren't in ascending
     * order.  The ARM ABI specifies that r0-r3 and r12, (also called ip), are
     * argument and scratch registers, and we also know that lr is invariably
     * the first register saved (because "ldmfd sp!,{...,lr}" is a very handy
     * return sequence).  It's also a good bet to look for in and out in
     * r0-r1, hence the registers used here. */
    #define REGISTER_BLOCK "{r2, r3, r4, ip, lr}"
    __asm__ volatile(
        "ldmia   %[in]!, "  REGISTER_BLOCK "\n\t"
        "stmia   %[out]!, " REGISTER_BLOCK "\n\t"
        "ldmia   %[in]!, "  REGISTER_BLOCK "\n\t"
        "stmia   %[out]!, " REGISTER_BLOCK
        : "=m"(out)
        : [in]"r"(in), [out]"r"(out), "m"(*in)
        : "r2", "r3", "r4", "ip", "lr" );
    #undef REGISTER_BLOCK
}



/* Helper macros for determining the underlying type for each supported
 * record type. */
#define TYPEOF(record)   TYPEOF_##record

#define TYPEOF_longin    int
#define TYPEOF_longout   int
#define TYPEOF_ai        int
#define TYPEOF_ao        int
#define TYPEOF_bi        bool
#define TYPEOF_bo        bool
#define TYPEOF_stringin  EPICS_STRING
#define TYPEOF_stringout EPICS_STRING
#define TYPEOF_mbbi      int
#define TYPEOF_mbbo      int


/* The three basic types, int, double, bool, are supported by corresponding
 * input and output records working through the reader and writer interfaces
 * defined above. */
#define DECLARE_INTERFACE(access, type) \
    typedef I_##access<TYPEOF(type)> I_##type

DECLARE_INTERFACE(READER, longin);
DECLARE_INTERFACE(WRITER, longout);
DECLARE_INTERFACE(READER, ai);
DECLARE_INTERFACE(WRITER, ao);
DECLARE_INTERFACE(READER, bi);
DECLARE_INTERFACE(WRITER, bo);
DECLARE_INTERFACE(READER, stringin);
DECLARE_INTERFACE(WRITER, stringout);
DECLARE_INTERFACE(READER, mbbi);
DECLARE_INTERFACE(WRITER, mbbo);



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



/*****************************************************************************/
/*                                                                           */
/*                        Publish EPICS Interfaces                           */
/*                                                                           */
/*****************************************************************************/

/* This macro is used to declare general purpose EPICS variable publishing
 * methods for publishing an I_<record> interface by name. */
#define DECLARE_PUBLISH(record) \
    void Publish_##record(const char * Name, I_##record & Record)
    

/* Declaration of Publish_<record> methods for each supported record type.
 * Every visible PV should be made available through a call to the appropriate
 * one of these, either directly or through a wrapper in publish.h. */
DECLARE_PUBLISH(longin);
DECLARE_PUBLISH(longout);
DECLARE_PUBLISH(ai);
DECLARE_PUBLISH(ao);
DECLARE_PUBLISH(bi);
DECLARE_PUBLISH(bo);
DECLARE_PUBLISH(stringin);
DECLARE_PUBLISH(stringout);
DECLARE_PUBLISH(mbbi);
DECLARE_PUBLISH(mbbo);
DECLARE_PUBLISH(waveform);
