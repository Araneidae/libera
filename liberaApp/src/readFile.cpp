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


/* A minimal EPICS 14 device. */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

//#include <sysLib.h>
#include <devSup.h>
#include <recSup.h>
#include <dbScan.h>
#include <epicsExport.h>

#include <longinRecord.h>
#include <longoutRecord.h>


#define OK 0




/*****************************************************************************/
/*                                                                           */
/*                             readFile Definition                           */
/*                                                                           */
/*****************************************************************************/


struct LONGIN_PARAM
{
    /* Counts number of (whitespace separated) fields to be skipped
     * before reading a value. */
    int SkipFields;
    int SkipLines;
    /* To save system resources, we re-open the file each time
     * through, so here we remember the file name. */
    const char * FileName;
};


static long init_record_longin(longinRecord *pli)
{
    LONGIN_PARAM & dpvt = * new LONGIN_PARAM;
    pli->dpvt = & dpvt;
    char * Inp = pli->inp.value.constantStr;
    dpvt.FileName = Inp;
    dpvt.SkipFields = 0;
    dpvt.SkipLines = 0;

    /* Now check for an optional delimiter after the filename.  We
     * can't use a space as the separator as, alas, this appears to
     * get eaten by EPICS as a field terminator.  Note that strsep
     * will also automatically fix up the file name. */
    strsep(& Inp, "|");
    if (Inp != NULL)
    {
        dpvt.SkipFields = atoi(Inp);
        strsep(&Inp, ",");
        if (Inp != NULL)
            dpvt.SkipLines = atoi(Inp);
    }
    
    return OK;
}




static long read_longin(longinRecord *pli)
{
    LONGIN_PARAM & dpvt = * (LONGIN_PARAM *) pli->dpvt;
    pli->val = 0;    /* Default value if reading fails. */

    int input = open(dpvt.FileName, O_RDONLY);
    if (input == -1)
    {
        fprintf(stderr, "%s: unable to open file \"%s\"\n",
            pli->name, dpvt.FileName);
        return -1;
    }
    else
    {
        /* A 1000 character line buffer is enough for our application. */
        char Line[1000];
        int rx = read(input, Line, sizeof(Line) - 1);
        close(input);

        bool ok = rx > 0;
        if (ok)
        {
            Line[rx] = '\0';
            char * Field = Line;

            /* First skip the require number of lines. */
            for (int i = 0;  ok  &&  i < dpvt.SkipLines;  i ++)
            {
                strsep(&Field, "\n");
                ok = Field != NULL;
            }
            
            /* Skip the required number of space delimited fields
             * before reading. */
            for (int i = 0; ok  &&  i < dpvt.SkipFields; i ++)
            {
                Field += strspn(Field, " ");
                strsep(&Field, " ");
                ok = Field != NULL;
            }
            if (ok)
                pli->val = atoi(Field);
        }
        
        if (!ok)
            fprintf(stderr, "%s: unable to read value\n", pli->name);
        return ok ? OK : -1;
    }
}



/*****************************************************************************/
/*                                                                           */
/*                            writeFile Definition                           */
/*                                                                           */
/*****************************************************************************/



struct LONGOUT_PARAM
{
    /* File name of file to write. */
    const char * FileName;
};


static long init_record_longout(longoutRecord *plo)
{
    LONGOUT_PARAM & dpvt = * new LONGOUT_PARAM;
    plo->dpvt = & dpvt;
    dpvt.FileName = plo->dol.value.constantStr;
    return OK;
}




static long write_longout(longoutRecord *plo)
{
    LONGOUT_PARAM & dpvt = * (LONGOUT_PARAM *) plo->dpvt;

    int output = open(dpvt.FileName, O_WRONLY);
    if (output == -1)
    {
        fprintf(stderr, "%s: unable to open file \"%s\"\n",
            plo->name, dpvt.FileName);
        return -1;
    }
    else
    {
        char Line[20];
        snprintf(Line, sizeof(Line), "%d", plo->val);
        int length = strlen(Line);

        bool ok = write(output, Line, length) == length;
        close(output);
        if (!ok)
            fprintf(stderr, "%s: unable to write value\n", plo->name);
        return ok ? OK : -1;
    }
}



/*****************************************************************************/
/*                                                                           */
/*                          Device Driver Definitions                        */
/*                                                                           */
/*****************************************************************************/

static struct {
    long number;
    long (*dev_report)(int);
    long (*init)(int);
    long (*init_record)(longinRecord *);
    long (*get_ioint_info)(int, longinRecord *, IOSCANPVT *);
    long (*read_longin)(longinRecord *);
} readFile =
{
    5,
    NULL,               /* Unused */
    NULL,               /* IOC initialisation */
    init_record_longin, /* Record initialisation */
    NULL,               /* Interrupt scanning */
        
    read_longin         /* Read value */
};



static struct {
    long number;
    long (*dev_report)(int);
    long (*init)(int);
    long (*init_record)(longoutRecord *);
    long (*get_ioint_info)(int, longoutRecord *, IOSCANPVT *);
    long (*write_longout)(longoutRecord *);
} writeFile =
{
    5,
    NULL,
    NULL,
    init_record_longout,
    NULL,
        
    write_longout
};


epicsExportAddress(dset, readFile);
epicsExportAddress(dset, writeFile);
