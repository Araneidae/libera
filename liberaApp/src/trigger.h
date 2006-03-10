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

#include <semaphore.h>

/* Simple trigger event notification to EPICS. */


/* A simple trigger class designed to publish I/O Intr events to EPICS. */

class TRIGGER : public I_bi
{
public:
    TRIGGER(bool InitialValue=true);

    /* This method is used to signal EPICS that this trigger is ready. */
    bool Ready();

    /* This changes the trigger value and also signals EPICS. */
    void Write(bool NewValue);

    /* Reads the currently set trigger value. */
    bool Read() { return Value; }

private:
    bool read(bool &Value);
    bool EnableIoIntr(I_INTR & Intr);

    /* Callback used to notify trigger event to EPICS. */
    I_INTR * iIntr;
    /* Internal value. */
    bool Value;
};


/* This class implements synchronisation with epics by publishing two
 * records, named "TRIG" and "DONE".  The database should be configured to
 * use these thus:
 *
 *      record(bi, "TRIG")
 *      {
 *          field(SCAN, "I/O Intr")
 *          field(FLNK, "FANOUT")
 *      }
 *      record(fanout, "FANOUT")
 *      {
 *          field(LNK1, "first-record")     # Process all associated
 *          ...                             # records here
 *          field(LNKn, "DONE")
 *      }
 *      record(bo, "DONE") { }
 *
 * In other words, TRIG should initiate processing on all records in its
 * group and then DONE should be processed to indicate that all processing is
 * complete.  The Libera epics driver will then block between signalling TRIG
 * and receiving receipt of DONE to ensure that the record processing block
 * retrieves a consistent set of data.
 *
 * The underling driver code should be of the form
 *
 *      while(running)
 *      {
 *          wait for event;
 *          Interlock.Wait();
 *          process data for epics;
 *          Interlock.Ready()
 *      }
 *
 * Note that Wait()ing is the first action: this is quite important. */

class INTERLOCK
{
public:
    INTERLOCK();

    /* This method actually publishes the trigger and done records. */
    void Publish(const char * Prefix);

    /* This signals EPICS that there is data to be read and sets the
     * interlock up ready to be read. */
    void Ready();

    /* This blocks until EPICS reports back by processing the DONE record.
     * The first call must be made before calling Ready() and will wait for
     * EPICS to finish initialising. */
    void Wait();

    /* Private really: don't use externally. */
    static void EpicsReady();

private:
    bool ReportDone(bool);

    bool Value;
    TRIGGER Trigger;
    sem_t Interlock;
    INTERLOCK *Next;
    
    static INTERLOCK *InterlockList;
};



/* Implements simple enable flag which can be controlled through the EPICS
 * interface. */

class ENABLE : I_bi, I_bo
{
public:
    ENABLE();
    void Publish(const char * Prefix);
    bool Enabled() { return Value; }
private:
    bool read(bool &);
    bool init(bool &);
    bool write(bool);
    bool Value;
    PERSISTENT_BOOL Persistent;
};


/* This needs to be called for initial startup synchronisation. */
bool InitialiseTriggers();
