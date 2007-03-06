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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fts.h>

#include <dbFldTypes.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"

#include "sensors.h"



/* Sensor variables. */

/* The internal temperature, fan speeds and motherboard voltages are all read
 * through CSPI. */
static cspi_health_t Health;

static int MemoryFree;  // Nominal memory free (free + cached - ramfs)
static int RamfsUsage;  // Number of bytes allocated in ram filesystems
static int Uptime;      // Machine uptime in seconds
static int CpuUsage;    // % CPU usage over the last sample interval
static int EpicsUp;     // EPICS run time in seconds

/* Supporting variables used for CPU calculation. */
static double LastUptime;
static double LastIdle;
static double EpicsStarted;



/* Helper routine for using scanf to parse a file. */

static bool ParseFile(
    const char * Filename, int Count, const char * Format, ...)
    __attribute__((format(scanf, 3, 4)));
static bool ParseFile(
    const char * Filename, int Count, const char * Format, ...)
{
    bool Ok = false;
    FILE * input = fopen(Filename, "r");
    if (input == NULL)
    {
        char Message[80];
        snprintf(Message, sizeof(Message), "Unable to open file %s", Filename);
        perror(Message);
    }
    else
    {
        va_list args;
        va_start(args, Format);
        Ok = vfscanf(input, Format, args) == Count;
        if (!Ok)
            printf("Error parsing %s\n", Filename);
        fclose(input);
    }
    return Ok;
}



/* Total uptime and idle time can be read directly from /proc/uptime, and by
 * keeping track of the cumulative idle time we can report percentage CPU
 * usage over the scan period. */

static void ProcessUptimeAndIdle()
{
    double NewUptime, NewIdle;
    if (ParseFile("/proc/uptime", 2, "%lf %lf", &NewUptime, &NewIdle))
    {
        Uptime = int(NewUptime);

        double SampleTime = NewUptime - LastUptime;
        double IdleTime = NewIdle - LastIdle;
        CpuUsage = int(1e5 * (1.0 - IdleTime / SampleTime));

        LastUptime = NewUptime;
        LastIdle = NewIdle;
        EpicsUp = int(NewUptime - EpicsStarted);
    }
}


static void InitialiseUptime()
{
    ParseFile("/proc/uptime", 1, "%lf", &EpicsStarted);
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
    int Free, Cached;
    if (ParseFile("/proc/meminfo", 2, "%*[^\n]\nMem: %*d %*d %d %*d %*d %d",
            &Free, &Cached))
    {
        RamfsUsage = FindRamfsUsage();
        MemoryFree = Free + Cached - RamfsUsage;
    }
}


static void ProcessSensors()
{
    ProcessUptimeAndIdle();
    ProcessFreeMemory();
    ReadHealth(Health);
}



#define PUBLISH_BLOCK(Type, BaseName, Array) \
    ( { \
        for (unsigned int i = 0; i < ARRAY_SIZE(Array); i ++) \
        { \
            char Number[10]; \
            sprintf(Number, "%d", i+1); \
            Publish_##Type(Concat(BaseName, Number), Array[i]); \
        } \
    } )


bool InitialiseSensors()
{
    Publish_longin("SE:TEMP",   Health.temp);
    PUBLISH_BLOCK(longin, "SE:FAN",  Health.fan);
    PUBLISH_BLOCK(ai,     "SE:VOLT", Health.voltage);

    Publish_ai("SE:FREE",    MemoryFree);
    Publish_ai("SE:RAMFS",   RamfsUsage);
    Publish_ai("SE:UPTIME",  Uptime);
    Publish_ai("SE:EPICSUP", EpicsUp);
    Publish_ai("SE:CPU",     CpuUsage);

    PUBLISH_ACTION("SE:PROCESS", ProcessSensors);

    InitialiseUptime();
    
    return true;
}
