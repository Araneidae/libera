/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2007  Michael Abbott, Diamond Light Source Ltd.
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


/* EPICS device driver interface to Libera.
 *
 * This file implements the generic device support required for interfacing
 * to the Libera Beam Position Monitor, but does not provide any direct
 * functionality.
 *
 * The following record types are supported:
 *      longin, longout, ai, ao, bi, bo, stringin, stringout, mbbi, mbbo,
 *      waveform. */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <devSup.h>
#include <recSup.h>
#include <dbScan.h>
#include <epicsExport.h>

#include <alarm.h>
#include <dbFldTypes.h>
#include <recGbl.h>
#include <dbCommon.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <waveformRecord.h>

#include "device.h"


/* Epics processing return codes. */
#define OK              0
#define ERROR           1
#define NO_CONVERT      2       // Special code for ai/ao conversion


//#define PRINTF(args...) printf(args)
#define PRINTF(args...)



/****************************************************************************/
/*                                                                          */
/*                         Generic Publish by Name                          */
/*                                                                          */
/****************************************************************************/

/* All records are published via a call to Publish_<record>, where <record>
 * is the appropriate record type, and discovered via an internally called
 * Search_<record> method.
 *    This interface is constructed here. */


/* A simple lookup table class. */

class LOOKUP
{
public:
    LOOKUP()
    {
        List = NULL;
    }

    /* Method to look up by name.  Returns NULL if not found. */
    I_RECORD * Find(const char * Name)
    {
        for (ENTRY * entry = List; entry != NULL; entry = entry->Next)
            if (strcmp(entry->Name, Name) == 0)
                return entry->Value;
        return NULL;
    }

    /* Inserts a new entry into the lookup table.  Note that the given string
     * is *NOT* copied, so the caller should ensure that it is persistent. */
    void Insert(const char * Name, I_RECORD * Value)
    {
        ENTRY * Entry = new ENTRY;
        Entry->Next = List;
        Entry->Name = Name;
        Entry->Value = Value;
        List = Entry;
    }

private:
    struct ENTRY
    {
        ENTRY * Next;
        const char * Name;
        I_RECORD * Value;
    };

    ENTRY * List;
};



/* This macro builds the appropriate Publish_<record> and Search_<record>
 * methods and initialises any associated static state. */
#define DEFINE_PUBLISH(record) \
    static LOOKUP Lookup_##record; \
    DECLARE_PUBLISH(record) \
    { \
        PRINTF("Publishing %s %s\n", Name, #record); \
        Lookup_##record.Insert(Name, &Record); \
    } \
    \
    static I_##record * Search_##record(const char *Name) \
    { \
        return dynamic_cast<I_##record *>(Lookup_##record.Find(Name)); \
    }


DEFINE_PUBLISH(longin);
DEFINE_PUBLISH(longout);
DEFINE_PUBLISH(ai);
DEFINE_PUBLISH(ao);
DEFINE_PUBLISH(bi);
DEFINE_PUBLISH(bo);
DEFINE_PUBLISH(stringin);
DEFINE_PUBLISH(stringout);
DEFINE_PUBLISH(mbbi);
DEFINE_PUBLISH(mbbo);
DEFINE_PUBLISH(waveform);




/*****************************************************************************/
/*                                                                           */
/*                        Common Record Implementation                       */
/*                                                                           */
/*****************************************************************************/


/* All record implementations use this class. */

class RECORD_BASE : public I_INTR
{
public:
    RECORD_BASE(I_RECORD &iRecord) :
        iRecord(iRecord)
    {
        ioscanpvt = NULL;
        IOSCANPVT newIoscanpvt;
        if (iRecord.EnableIoIntr(*this))
        {
            scanIoInit(&newIoscanpvt);
            ioscanpvt = newIoscanpvt;
        }
    }
    
    /* Callback routine called (possibly in signal handler context, or in an
     * arbitrary thread) to notify that I/O Intr processing should occur. */
    bool IoIntr()
    {
        if (ioscanpvt == NULL)
            return false;
        else
        {
            scanIoRequest(ioscanpvt);
            return true;
        }
    }

    /* Used by record implementation to implement get_ioint functionality. */
    void GetIoInt(IOSCANPVT *pIoscanpvt)
    {
        *pIoscanpvt = ioscanpvt;
    }

    inline I_RECORD * GetRecord() { return & iRecord; }

    
private:
    I_RECORD & iRecord;
    IOSCANPVT ioscanpvt;
};




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                               I_WAVEFORM                                  */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Simple helper class for routine I_waveform implementations: implements
 * array field type validation. */


I_WAVEFORM::I_WAVEFORM(epicsEnum16 Type) :
    Type(Type)
{
}

bool I_WAVEFORM::BindRecord(dbCommon * p)
{
    waveformRecord * pr = (waveformRecord *) p;
    /* Check that the types match. */
    if (pr->ftvl == Type)
        return true;
    else
    {
        printf("Array FTVL mismatch %d != %d\n", Type, pr->ftvl);
        return false;
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Common Device Driver Routines                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* Common I/O Intr scanning support: uses the fact that pr->dpvt always
 * contains the appropriate GetIoInt implementation. */

static long get_ioint_(int, dbCommon *pr, IOSCANPVT *pIoscanpvt) 
{ 
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; 
    if (base == NULL) 
        return ERROR; 
    else 
    {       
        base->GetIoInt(pIoscanpvt); 
        return OK; 
    } 
}



/* Common record initialisation.  Performs the appropriate record
 * initialisation once the record implementation has been found. */

static bool init_record_(
    const char * RecordType, const char * Name,
    dbCommon *pr, I_RECORD *iRecord)
{
    /* If we successfully found a record then try binding to it.  If this
     * fails then record initialisation fails, otherwise we're done. */
    if (iRecord == NULL)
    {
        printf("Libera record %s:%s not found\n", RecordType, Name);
        return false;
    }
    else if (!iRecord->BindRecord(pr))
    {
        printf("Error binding libera record %s:%s\n", RecordType, Name);
        return false;
    }
    else
    {
        pr->dpvt = new RECORD_BASE(*iRecord);
        return true;
    }
}



static void SetTimestamp(dbCommon *pr, struct timespec &Timestamp)
{
    /* Convert the standard Unix timespec value into the EPICS epoch
     * timestamp (subtract 20 years). */
    epicsTimeFromTimespec(&pr->time, &Timestamp);
}


/* Record initialisation post processed: ensures that the EPICS data
 * structures are appropriately initialised.  The data has already been read,
 * but we still need to set up the alarm state and give the data a sensible
 * initial timestamp. */

static void post_init_record_out(dbCommon *pr, I_RECORD *iRecord)
{
    (void) recGblSetSevr(pr, READ_ALARM, iRecord->AlarmStatus());
    recGblResetAlarms(pr); 
    struct timespec Timestamp;
    if (!iRecord->GetTimestamp(Timestamp))
        /* If the record doesn't have its own timestamp then synthesise one
         * instead from the real time clock. */
        clock_gettime(CLOCK_REALTIME, & Timestamp);
    SetTimestamp(pr, Timestamp);
}



/* Common record post-processing.  Updates the alarm state as appropriate,
 * and checks if the timestamp should be written here. */

static void post_process(dbCommon *pr, epicsEnum16 nsta, I_RECORD *iRecord)
{
    (void) recGblSetSevr(pr, nsta, iRecord->AlarmStatus());
    struct timespec Timestamp;
    if (iRecord->GetTimestamp(Timestamp))
        SetTimestamp(pr, Timestamp);
}



/*****************************************************************************/
/*                                                                           */
/*                    Boilerplate generation support.                        */
/*                                                                           */
/*****************************************************************************/


/* Reads a value from the appropriate record interface.  An intermediate
 * value is used so that the record interface type doesn't need to precisely
 * match the data type stored in the EPICS record. */
#define READ_ADAPTER(record, action, field) \
    ( { \
        TYPEOF(record) Value; \
        bool Ok = action(Value); \
        field = Value; \
        Ok; \
    } )

#define READ_DIRECT(record, action, field) \
    action(field)

/* This macro expands to either READ_DIRECT or READ_ADAPTER, depending on how
 * the data from this record needs to be processed. */
#define READ_VALUE(record, action, field) \
    READ_##record(record, action, field)

#define WRITE_VALUE(record, action, field) \
    action(field)


/* Record initialisation is simply a matter of constructing an instance of
 * the appropriate record type. */
#define INIT_RECORD(record, inOrOut) \
    static long init_record_##record(record##Record *pr) \
    { \
        const char * Name = pr->inOrOut.value.constantStr; \
        if (init_record_( \
                #record, Name, (dbCommon *) pr, Search_##record(Name))) \
            POST_INIT_##inOrOut(record, pr) \
        else \
            return ERROR; \
    }


/* For inp records there is no special post initialisation processing. */
#define POST_INIT_inp(record, pr)   return OK;

/* For out records we need to read the current underlying device value as
 * part of initialisation.  The precise processing performed depends on the
 * record type: this is hooked in by defining POST_INIT_##record. */
#define POST_INIT_out(record, pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        pr->udf = ! READ_VALUE(record, iRecord->init, pr->VAL(record)); \
        post_init_record_out((dbCommon*)pr, iRecord); \
        return OK; \
    }



#define DEFINE_DEVICE(record, inOrOut, length, args...) \
    INIT_RECORD(record, inOrOut) \
    record##Device record##Libera = \
    { \
        length, \
        NULL, \
        NULL, \
        init_record_##record, \
        get_ioint_, \
        args \
    }; \
    epicsExportAddress(dset, record##Libera)


/* Helper code for extracting the appropriate I_record from the record. */
#define GET_RECORD(record, pr, var) \
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; \
    if (base == NULL) \
        return ERROR; \
    I_##record * var = dynamic_cast<I_##record *> (base->GetRecord())

#define SET_ALARM(pr, ACTION, iRecord) \
    (void) recGblSetSevr(pr, ACTION##_ALARM, iRecord->AlarmStatus())



/* Standard boiler-plate default record processing action.  The val field is
 * either read or written and the alarm state is set by interrogating the
 * record interface.  This processing is adequate for most record types. */
#define DEFINE_DEFAULT_PROCESS(record, action, ACTION, DO_ACTION) \
    static long action##_##record(record##Record * pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        bool Ok = DO_ACTION(record, iRecord->action, pr->VAL(record)); \
        post_process((dbCommon *)pr, ACTION##_ALARM, iRecord); \
        return Ok ? OK : ERROR; \
    }


#define DEFINE_DEFAULT_READ(record) \
    DEFINE_DEFAULT_PROCESS(record, read, READ, READ_VALUE) 
#define DEFINE_DEFAULT_WRITE(record) \
    DEFINE_DEFAULT_PROCESS(record, write, WRITE, WRITE_VALUE) 



/*****************************************************************************/
/*                                                                           */
/*                        Device Driver Implementations                      */
/*                                                                           */
/*****************************************************************************/


/* For some types we read and write the VAL field, for others the RVAL field.
 * Here we define which. */
#define longin_VAL      val
#define longout_VAL     val
#define ai_VAL          rval
#define ao_VAL          rval
#define bi_VAL          rval
#define bo_VAL          rval
#define stringin_VAL    val
#define stringout_VAL   val
#define mbbi_VAL        rval
#define mbbo_VAL        rval

#define VAL(record)     record##_VAL

/* Type adapters.  Some types need to be read directly, others need to be
 * read via the read adapter. */

#define READ_longin     READ_DIRECT
#define READ_longout    READ_DIRECT
#define READ_ai         READ_DIRECT
#define READ_ao         READ_DIRECT
#define READ_bi         READ_ADAPTER
#define READ_bo         READ_ADAPTER
#define READ_stringin   READ_DIRECT
#define READ_stringout  READ_DIRECT
#define READ_mbbi       READ_ADAPTER
#define READ_mbbo       READ_ADAPTER




/* Mostly we can use simple boilerplate for the process routines. */
DEFINE_DEFAULT_READ (longin)
DEFINE_DEFAULT_WRITE(longout)
DEFINE_DEFAULT_READ (ai)
DEFINE_DEFAULT_WRITE(ao)
DEFINE_DEFAULT_READ (bi)
DEFINE_DEFAULT_WRITE(bo)
DEFINE_DEFAULT_READ (stringin)
DEFINE_DEFAULT_WRITE(stringout)
DEFINE_DEFAULT_READ (mbbi)
DEFINE_DEFAULT_WRITE(mbbo)


/* Reading a waveform doesn't fit into the fairly uniform pattern established
 * for the other record types. */

static long read_waveform(waveformRecord * pr)
{
    GET_RECORD(waveform, pr, i_waveform);
    pr->nord = i_waveform->read(pr->bptr, pr->nelm);
    post_process((dbCommon *)pr, READ_ALARM, i_waveform);
    /* Note, by the way, that the waveform record support carefully ignores
     * my return code! */
    return pr->nord > 0 ? OK : ERROR;
}


/* Also need dummy special_linconv routines for ai and ao. */

static long linconv_ai(aiRecord *, int) { return OK; }
static long linconv_ao(aoRecord *, int) { return OK; }



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          Device Driver Definitions                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "recordDevice.h"

DEFINE_DEVICE(longin,    inp, 5, read_longin);
DEFINE_DEVICE(longout,   out, 5, write_longout);
DEFINE_DEVICE(ai,        inp, 6, read_ai,  linconv_ai);
DEFINE_DEVICE(ao,        out, 6, write_ao, linconv_ao);
DEFINE_DEVICE(bi,        inp, 5, read_bi);
DEFINE_DEVICE(bo,        out, 5, write_bo);
DEFINE_DEVICE(stringin,  inp, 5, read_stringin);
DEFINE_DEVICE(stringout, out, 5, write_stringout);
DEFINE_DEVICE(mbbi,      inp, 5, read_mbbi);
DEFINE_DEVICE(mbbo,      out, 5, write_mbbo);
DEFINE_DEVICE(waveform,  inp, 5, read_waveform);
