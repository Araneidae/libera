/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
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


/* Simple trigger event notification to EPICS and related functionality. */


/* A simple trigger class designed to publish I/O Intr events to EPICS
 * together with timestamps. */

class TRIGGER : public UPDATER_bool
{
public:
    TRIGGER(bool InitialValue = true);

    /* This method is used to signal EPICS that this trigger is ready.  If no
     * timestamp is given then we use the current time. */
    void Ready(const struct timespec *Timestamp = NULL);

private:
    bool GetTimestamp(struct timespec & Result);
    struct timespec Timestamp;
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

    /* This method actually publishes the trigger and done records.  Their
     * default names can be overridden if required. */
    void Publish(
        const char * Prefix, bool PublishMC = false,
        const char * TrigName = NULL, const char * DoneName = NULL);

    /* This signals EPICS that there is data to be read and sets the
     * interlock up ready to be read. */
    void Ready(
        const LIBERA_TIMESTAMP &NewTimestamp = *(LIBERA_TIMESTAMP*)NULL);

    /* This blocks until EPICS reports back by processing the DONE record.
     * The first call must be made before calling Ready() and will wait for
     * EPICS to finish initialising. */
    void Wait();

private:
    bool ReportDone(int);

    int Value;
    int MachineClockLow;
    int MachineClockHigh;
    TRIGGER Trigger;
    SEMAPHORE Interlock;
    const char * Name;
};



/* Implements simple persistent enable flag which can be controlled through
 * the EPICS interface. */

class ENABLE : I_bo
{
public:
    ENABLE();
    void Publish(const char * Prefix);
    bool Enabled() { return Value; }
private:
    bool init(bool &);
    bool write(bool);
    bool Value;
    PERSISTENT_BOOL Persistent;
};


/* This needs to be called for initial startup synchronisation. */
bool InitialiseTriggers();
