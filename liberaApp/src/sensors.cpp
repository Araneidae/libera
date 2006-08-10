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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fts.h>

#include <dbFldTypes.h>

#include "drivers.h"
#include "persistent.h"
#include "publish.h"

#include "sensors.h"



/* Sensor variables. */
static int Temperature; // Internal case temperature
static int FanSpeed1;   // Fan speeds
static int FanSpeed2;
static int MemoryFree;  // Nominal memory free (free + cached - ramfs)
static int RamfsUsage;  // Number of bytes allocated in ram filesystems
static int Uptime;      // Machine uptime in seconds
static int CpuUsage;    // % CPU usage over the last sample interval

/* Supporting variables used for CPU calculation. */
static double LastUptime;
static double LastIdle;



/* Class to support reading fields from a file. */

class FILE_PARSER
{
public:
    FILE_PARSER(const char * FileName) :
        FileName(FileName),
        Input(fopen(FileName, "r"))
    {
        Field = NULL;
        NextField = NULL;
        LineNumber = 0;
        FieldNumber = 0;
        
        Ok = Input != NULL;
        if (!Ok)
            printf("Unable to open file \"%s\"\n", FileName);

        /* If the file was successfully open, prime the pump by reading the
         * first line. */
        if (Ok)
            ReadLine();
    }

    ~FILE_PARSER()
    {
        if (Input != NULL)
            fclose(Input);
    }

    bool Good() { return Ok; }

    bool SkipLines(int Lines)
    {
        for (int i = 0; Ok && i < Lines; i ++)
            if (!ReadLine())
                printf("Run out of lines skipping %d (%d) in \"%s\"\n",
                    Lines, i, FileName);
        return Ok;
    }

    bool SkipFields(int Fields)
    {
        for (int i = 0; Ok && i < Fields; i ++)
            if (!ReadField())
                printf("Run out of fields skipping %d (%d) in \"%s\"\n",
                    Fields, i, FileName);
        return Ok;
    }

    bool ReadInt(int &Result)
    {
        if (ReadField())
        {
            char * End;
            Result = strtol(Field, &End, 10);
            Ok = Field < End  &&  *End == '\0';
            if (!Ok)
                printf("Field \"%s\" in \"%s\" is not a good integer\n",
                    Field, FileName);
        }
        return Ok;
    }
    
    bool ReadDouble(double &Result)
    {
        if (ReadField())
        {
            char * End;
            Result = strtod(Field, &End);
            Ok = Field < End  &&  *End == '\0';
            if (!Ok)
                printf("Field \"%s\" in \"%s\" is not a good double\n",
                    Field, FileName);
        }
        return Ok;
    }

    bool Expect(const char *Expectation)
    {
        if(ReadField())
        {
            Ok = strcmp(Field, Expectation) == 0;
            if (!Ok)
                printf("Expected \"%s\", got \"%s\", "
                    "field %d, line %d in \"%s\"\n",
                    Expectation, Field, FieldNumber, LineNumber, FileName);
        }
        return Ok;
    }

    /* If this call is successful, the returned string is valid until the
     * next operation on this class. */
    bool ReadString(const char *&Result)
    {
        if (ReadField())
            Result = Field;
        return Ok;
    }


private:
    bool ReadLine()
    {
        if (Ok)
        {
            LineNumber += 1;
            Ok = fgets(Line, sizeof(Line), Input) == Line;
            if (Ok)
            {
                /* Trim the trailing \n and set up for reading fields from
                 * this line. */
                int Length = strlen(Line);
                if (Length > 0  &&  Line[Length - 1] == '\n')
                    Line[Length - 1] = '\0';
                NextField = Line;
                FieldNumber = 0;
                Field = NULL;
            }
            else
                printf("Error reading line %d in \"%s\"\n",
                    LineNumber, FileName);
        }
        return Ok;
    }

    bool ReadField()
    {
        if (Ok)
        {
            FieldNumber += 1;
            NextField += strspn(NextField, " ");
            Field = strsep(&NextField, " ");
            Ok = Field != NULL;
            if (!Ok)
                printf("Unable to read field %d on line %d in \"%s\"\n",
                    FieldNumber, LineNumber, FileName);
        }
        return Ok;
    }

    
    bool Ok;
    char Line[1024];
    char *Field, *NextField;
    int LineNumber;
    int FieldNumber;
    
    const char * const FileName;
    FILE * const Input;
};




static bool ReadFileInt(
    int &Result, const char * FileName,
    int SkipFields = 0, int SkipLines = 0)
{
    FILE_PARSER Parser(FileName);
    return
        Parser.SkipLines(SkipLines) &&
        Parser.SkipFields(SkipFields) &&
        Parser.ReadInt(Result);
}



/* Total uptime and idle time can be read directly from /proc/uptime, and by
 * keeping track of the cumulative idle time we can report percentage CPU
 * usage over the scan period. */

static void ProcessUptimeAndIdle()
{
    FILE_PARSER Parser("/proc/uptime");
    double NewUptime, NewIdle;
    if (Parser.ReadDouble(NewUptime)  && Parser.ReadDouble(NewIdle))
    {
        Uptime = int(NewUptime);

        double SampleTime = NewUptime - LastUptime;
        double IdleTime = NewIdle - LastIdle;
        CpuUsage = int(1e5 * (1.0 - IdleTime / SampleTime));

        LastUptime = NewUptime;
        LastIdle = NewIdle;
    }
}


/* This discovers how many bytes of space are being consumed by the ramfs:
 * this needs to be subtracted from the "cached" space.
 *
 * We do this by walking all of the file systems mounted as ramfs -- the
 * actual set of mount points is hard-wired here. */

static int FindRamfsUsage()
{
    /* The following mount points all contain ram file systems. */
    static char * const RamFileSystems[] =
    {
        "/var/log",
        "/var/lock",
        "/var/run",
        "/tmp",
        NULL
    };

    FTS * fts = fts_open(RamFileSystems, FTS_PHYSICAL | FTS_XDEV, NULL);
    if (fts == NULL)
    {
        perror("Unable to open ram file systems");
        return 0;
    }

    int total = 0;
    FTSENT *ftsent;
    while (ftsent = fts_read(fts),  ftsent != NULL)
        if (ftsent->fts_info != FTS_D)
            total += ftsent->fts_statp->st_size;

    fts_close(fts);
        
    return total;
}


/* Free memory processing is a little tricky.  By reading /proc/meminfo we
 * can discover "free" and "cached" memory, but turning this into a true free
 * memory number is more difficult.
 *    In general, the cached memory is effectively free ... but
 * unfortunately, files in the RAM file system also appear as "cached" and
 * are NOT free.  Even more unfortunately, it appears to be particularly
 * difficult to how much space is used by the RAM file system! */

static void ProcessFreeMemory()
{
    /* First read the free and cached values. */
    FILE_PARSER Parser("/proc/meminfo");
    int Free, Cached;
    if (Parser.SkipLines(1)   &&  Parser.Expect("Mem:")  &&
        Parser.SkipFields(2)  &&  Parser.ReadInt(Free)  &&
        Parser.SkipFields(2)  &&  Parser.ReadInt(Cached))
    {
        RamfsUsage = FindRamfsUsage();
        MemoryFree = Free + Cached - RamfsUsage;
    }
}


static bool ProcessSensors(bool&Value)
{
    ProcessUptimeAndIdle();

    ProcessFreeMemory();

    /* Temperatures and fan speeds are just read directly from the
     * appropriate procfs files. */
    ReadFileInt(Temperature,
        "/proc/sys/dev/sensors/max1617a-i2c-0-29/temp1", 2);
    ReadFileInt(FanSpeed1,
        "/proc/sys/dev/sensors/max6650-i2c-0-48/fan1");
    ReadFileInt(FanSpeed2,
        "/proc/sys/dev/sensors/max6650-i2c-0-4b/fan1");
    
    Value = true;
    return true;
}


bool InitialiseSensors()
{
    Publish_longin("SE:TEMP",   Temperature);
    Publish_longin("SE:FAN1",   FanSpeed1);
    Publish_longin("SE:FAN2",   FanSpeed2);

    Publish_ai("SE:FREE",   MemoryFree);
    Publish_ai("SE:RAMFS",  RamfsUsage);
    Publish_ai("SE:UPTIME", Uptime);
    Publish_ai("SE:CPU",    CpuUsage);

    PUBLISH_FUNCTION_IN(bi, "SE:PROCESS", ProcessSensors);
    
    return true;
}
