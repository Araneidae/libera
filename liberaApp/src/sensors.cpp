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
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <fts.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"

#include "sensors.h"



/* We poll the sensors every 10 seconds. */
#define SENSORS_POLL_INTERVAL   10


/* Sensor variables. */

static int SystemTemperature;
static int FanSpeeds[2];
static int SystemVoltages[8];

static int MemoryFree;  // Nominal memory free (free + cached - ramfs)
static int RamfsUsage;  // Number of bytes allocated in ram filesystems
static int Uptime;      // Machine uptime in seconds
static int CpuUsage;    // % CPU usage over the last sample interval
static int EpicsUp;     // EPICS run time in seconds

/* Supporting variables used for CPU calculation. */
static double LastUptime;
static double LastIdle;
static double EpicsStarted;

/* The paths to the fan and temperature sensors need to be determined at
 * startup. */
static const char *proc_temp;
static const char *proc_fan0;
static const char *proc_fan1;
/* This records whether we're reading from /sys or /proc. */
static bool UseSys;


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



/* This helper routine is used to read a specific line from the /proc/meminfo
 * file: it scans for a line of the form
 *      <Prefix>   <Result> kB
 * and returns the integer result. */

static bool ReadMeminfoLine(FILE *MemInfo, const char *Prefix, int &Result)
{
    const int PrefixLength = strlen(Prefix);
    char Line[1024];
    while (fgets(Line, sizeof(Line), MemInfo))
    {
        if (strncmp(Line, Prefix, PrefixLength) == 0)
        {
            /* Good: this is our line. */
            if (sscanf(Line + PrefixLength, " %d ", &Result) == 1)
                return true;
            else
            {
                printf("Malformed /proc/meminfo line:\n\t\"%s\"\n", Line);
                return false;
            }
        }
    }
    /* Oops.  Couldn't find anything. */
    printf("Unable to find \"%s\" line in /proc/meminfo\n", Prefix);
    return false;
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
    FILE * MemInfo = fopen("/proc/meminfo", "r");
    if (MemInfo == NULL)
        perror("Unable to open /proc/meminfo");
    else
    {
        int Free, Cached;
        if (ReadMeminfoLine(MemInfo, "MemFree:", Free)  &&
            ReadMeminfoLine(MemInfo, "Cached:",  Cached))
        {
            RamfsUsage = FindRamfsUsage();
            MemoryFree = 1024 * (Free + Cached) - RamfsUsage;
        }
        fclose(MemInfo);
    }
}



/* The following reads the key system health parameters directly from the
 * appropriate devices and proc/sys files. */

static void ReadHealth()
{
    /* Annoyingly the format of the temperature readout depends on which
     * system version we're using! */
    ParseFile(proc_temp, 1,
        UseSys ? "%d" : "%*d\t%*d\t%d", &SystemTemperature);
    if (UseSys)
        SystemTemperature /= 1000;

    ParseFile(proc_fan0, 1, "%d", &FanSpeeds[0]);
    ParseFile(proc_fan1, 1, "%d", &FanSpeeds[1]);

    /* The system voltages are read directly from the msp device in binary
     * format.  This particular step takes a surprisingly long time (about
     * half a second) -- in particular, this one step requires all our
     * processing to be done in the sensors thread (rather than the
     * alternative of using an EPICS SCAN thread). */
    int msp = open("/dev/msp0", O_RDONLY);
    if (msp == -1)
        perror("Unable to open MSP device");
    else
    {
        read(msp, SystemVoltages, sizeof(SystemVoltages));
        close(msp);
    }
}



static void ProcessSensors()
{
    ProcessUptimeAndIdle();
    ProcessFreeMemory();
    ReadHealth();
}



class SENSORS_THREAD : public THREAD
{
public:
    SENSORS_THREAD() : THREAD("Sensors")
    {
        Interlock.Publish("SE");
    }
    
private:
    void Thread()
    {
        StartupOk();
        
        while (Running())
        {
            Interlock.Wait();
            ProcessSensors();
            Interlock.Ready();
            sleep(SENSORS_POLL_INTERVAL);
        }
    }

    INTERLOCK Interlock;
};



static SENSORS_THREAD * SensorsThread = NULL;


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
    /* Figure out where to read our fan and temperature sensors: under Linux
     * 2.6 we read from the /sys file system, but under 2.4 we read from /proc
     * instead. */
    UseSys = access("/sys", F_OK) == 0;
    if (UseSys)
    {
        /* The /sys file system exists.  All our sensors live here. */
        proc_temp = "/sys/class/i2c-adapter/i2c-0/device/0-0029/temp1_input";
        proc_fan0 = "/sys/class/i2c-adapter/i2c-0/device/0-004b/fan1_input";
        proc_fan1 = "/sys/class/i2c-adapter/i2c-0/device/0-0048/fan1_input";
    }
    else
    {
        /* No /sys file system: revert to the older /proc filesystem. */
        proc_temp = "/proc/sys/dev/sensors/max1617a-i2c-0-29/temp1";
        proc_fan0 = "/proc/sys/dev/sensors/max6650-i2c-0-4b/fan1";
        proc_fan1 = "/proc/sys/dev/sensors/max6650-i2c-0-48/fan1";
    }
    
    Publish_longin("SE:TEMP",   SystemTemperature);
    PUBLISH_BLOCK(longin, "SE:FAN",  FanSpeeds);
    PUBLISH_BLOCK(ai,     "SE:VOLT", SystemVoltages);

    Publish_ai("SE:FREE",    MemoryFree);
    Publish_ai("SE:RAMFS",   RamfsUsage);
    Publish_ai("SE:UPTIME",  Uptime);
    Publish_ai("SE:EPICSUP", EpicsUp);
    Publish_ai("SE:CPU",     CpuUsage);

    InitialiseUptime();

    SensorsThread = new SENSORS_THREAD();
    return SensorsThread->StartThread();
}



void TerminateSensors()
{
    if (SensorsThread != NULL)
        SensorsThread->Terminate();
}
