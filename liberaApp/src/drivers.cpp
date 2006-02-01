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


/* EPICS device driver interface to Libera.
 *
 * This file implements the generic device support required for interfacing
 * to the Libera Beam Position Monitor, but does not provide any direct
 * functionality.
 *
 * The following record types are supported:
 *      longin, longout, ai, ao, bi, bo, waveform. */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <devSup.h>
#include <recSup.h>
#include <dbScan.h>
#include <epicsExport.h>

#include <alarm.h>
#include <db_access.h>
#include <recGbl.h>
#include <dbCommon.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <waveformRecord.h>

#include "drivers.h"
#include "publish.h"


#define OK 0
#define ERROR 1



/*****************************************************************************/
/*                                                                           */
/*                          Common Record Implementation                     */
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
    void IoIntr()
    {
        if (ioscanpvt != NULL)
            scanIoRequest(ioscanpvt);
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
/*                          Device Driver Implementations                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/* Common I/O Intr scanning support: uses the fact that pr->dpvt always
 * contains the appropriate GetIoInt implementation. */

static long get_ioint_ (int, dbCommon *pr, IOSCANPVT *pIoscanpvt) 
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

static long init_record_(
    const char * RecordType, const char * Name,
    dbCommon *pr, I_RECORD *iRecord)
{
    /* If we successfully found a record then try binding to it.  If this
     * fails then record initialisation fails, otherwise we're done. */
    if (iRecord == NULL)
    {
        printf("Libera record %s:%s not found\n", RecordType, Name);
        return ERROR;
    }
    else if (!iRecord->BindRecord(pr))
    {
        printf("Error binding libera record %s:%s\n", RecordType, Name);
        return ERROR;
    }
    else
    {
        pr->dpvt = new RECORD_BASE(*iRecord);
        return OK;
    }
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                    Boilerplate generation support.                        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Record initialisation is simply a matter of constructing an instance of
 * the appropriate record type. */
#define INIT_RECORD(record, inOrOut) \
    static long init_record_##record(record##Record *pr) \
    { \
        const char * Name = pr->inOrOut.value.constantStr; \
        return init_record_( \
            #record, Name, (dbCommon *) pr, Search_##record(Name)); \
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


#define GET_RECORD(record, pr, var) \
    RECORD_BASE * base = (RECORD_BASE *) pr->dpvt; \
    if (base == NULL) \
        return ERROR; \
    I_##record * var = dynamic_cast<I_##record *> (base->GetRecord())

#define SET_ALARM(pr, ACTION, iRecord) \
    recGblSetSevr(pr, ACTION##_ALARM, iRecord->AlarmStatus())


/* Standard boiler-plate default record processing action.  The val field is
 * either read or written and the alarm state is set by interrogating the
 * record interface.  This processing is adequate for most record types. */
#define DEFINE_DEFAULT_PROCESS(record, action, ACTION) \
    static long action##_##record(record##Record * pr) \
    { \
        GET_RECORD(record, pr, iRecord); \
        bool Ok = iRecord->action(pr->val); \
        SET_ALARM(pr, ACTION, iRecord); \
        return Ok ? OK : ERROR; \
    }



/*****************************************************************************/
/*                                                                           */
/*                          Device Driver Implementations                    */
/*                                                                           */
/*****************************************************************************/

/* Record specific access support. */


/* Mostly we can use simple boilerplate for the process routines. */

DEFINE_DEFAULT_PROCESS(longin, read, READ)
DEFINE_DEFAULT_PROCESS(longout, write, WRITE)
DEFINE_DEFAULT_PROCESS(ao, write, WRITE)
DEFINE_DEFAULT_PROCESS(bo, write, WRITE)



/* The following three routines need slightly special handling. */


static long read_ai(aiRecord *pr)
{
    GET_RECORD(ai, pr, i_ai);
    bool Ok = i_ai->read(pr->val);
    /* Because we are dealing with doubles already we want to suppress the
     * construction of val from rval.  We therefore have to set udf ourselves
     * (emulating what is done in aiRecord.c:convert) and return 2 to suppress
     * conversion. */
    pr->udf = isnan(pr->val);
    SET_ALARM(pr, READ, i_ai);
    return Ok ? 2 : ERROR;
}


static long read_bi(biRecord * pr)
{
    GET_RECORD(bi, pr, i_bi);
    bool Flag;
    bool Ok = i_bi->read(Flag);
    pr->rval = Flag;
    SET_ALARM(pr, READ, i_bi);
    return Ok ? OK : ERROR;
}


static long read_waveform(waveformRecord * pr)
{
    GET_RECORD(waveform, pr, i_waveform);
    pr->nord = i_waveform->read(pr->bptr, pr->nelm);
    SET_ALARM(pr, READ, i_waveform);
    /* Note, by the way, that the waveform record support carefully ignores
     * my return code! */
    return pr->nord > 0 ? OK : ERROR;
}


/* Also need dummy special_linconv routines for ai and ao. */

static long linconv_ai(aiRecord *, int) { return OK; }
static long linconv_ao(aoRecord *, int) { return OK; }



/*****************************************************************************/
/*                                                                           */
/*                          Device Driver Definitions                        */
/*                                                                           */
/*****************************************************************************/

#include "recordDevice.h"

DEFINE_DEVICE(longin,   inp, 5, read_longin);
DEFINE_DEVICE(longout,  out, 5, write_longout);
DEFINE_DEVICE(ai,       inp, 6, read_ai, linconv_ai);
DEFINE_DEVICE(ao,       out, 6, write_ao, linconv_ao);
DEFINE_DEVICE(bi,       inp, 5, read_bi);
DEFINE_DEVICE(bo,       out, 5, write_bo);
DEFINE_DEVICE(waveform, inp, 5, read_waveform);
