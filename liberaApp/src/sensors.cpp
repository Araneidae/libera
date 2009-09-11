/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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
#include <sys/select.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <fts.h>
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "versions.h"
#include "healthd.h"

#include "sensors.h"



/* We poll the sensors every 10 seconds. */
#define SENSORS_POLL_INTERVAL   10


/* Sensor variables. */

static int RfTemperature1, RfTemperature2, MbTemperature;
static int FanSpeeds[2];
static int FanSetSpeeds[2];
static int SystemVoltages[8];

static int MemoryFree;  // Nominal memory free (free + cached - ramfs)
static int RamfsUsage;  // Number of bytes allocated in ram filesystems
static int Uptime;      // Machine uptime in seconds
static int CpuUsage;    // % CPU usage over the last sample interval
static int EpicsUp;     // EPICS run time in seconds

/* Sensors can be disabled for particularly quiet operation. */
static bool EnableSensors = true;
static int TargetTemperature = 40;


/* Supporting variables used for CPU calculation. */
static double LastUptime;
static double LastIdle;
static double EpicsStarted;

/* The paths to the fan and temperature sensors need to be determined at
 * startup. */
static const char *proc_temp_rf1;   // RF board temperatures
static const char *proc_temp_rf2;
static const char *proc_temp_mb;    // Motherboard temperature
static const char *proc_fan0;       // Fan measured speeds
static const char *proc_fan1;
static const char *proc_fan0_set;   // Fan programmed speeds
static const char *proc_fan1_set;
/* This records whether we're reading from /sys or /proc. */
static bool UseSys;


static char ** RamFileSystems;      // List of file systems to scan for files


/* Helper routine for using scanf to parse a file. */

static bool ParseFile(
    const char * Filename, int Count, const char * Format, ...)
    __attribute__((format(scanf, 3, 4)));
static bool ParseFile(
    const char * Filename, int Count, const char * Format, ...)
{
    FILE * input;
    bool Ok = TEST_NULL(input = fopen(Filename, "r"));
    if (Ok)
    {
        va_list args;
        va_start(args, Format);
        Ok = TEST_OK(vfscanf(input, Format, args) == Count);
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


static bool InitialiseRamfsUsage()
{
    const char * string;
    bool Ok = TEST_NULL(string = getenv("TEMP_FS_LIST"));
    if (Ok)
    {
        char * ramfs_list = (char *) malloc(strlen(string) + 1);
        strcpy(ramfs_list, string);

        /* Split the string up and count the number of entries. */
        int ramfs_count = 1;
        for (char * ramfs = strchr(ramfs_list, ' '); ramfs != NULL;
             ramfs = strchr(ramfs, ' '))
        {
            *ramfs++ = '\0';
            ramfs_count += 1;
        }

        /* Assemble the final list of file systems for fts_... to scan. */
        RamFileSystems = (char **) malloc((ramfs_count + 1) * sizeof(char *));
        for (int i = 0; i < ramfs_count; i ++)
        {
            RamFileSystems[i] = ramfs_list;
            ramfs_list += strlen(ramfs_list) + 1;
        }
        RamFileSystems[ramfs_count] = NULL;
    }
    return Ok;
}

/* This discovers how many bytes of space are being consumed by the ramfs:
 * this needs to be subtracted from the "cached" space.
 *
 * We do this by walking all of the file systems mounted as ramfs -- the
 * actual set of mount points must be set in TEMP_FS_LIST. */

static int FindRamfsUsage()
{
    FTS * fts;
    if (TEST_NULL(fts = fts_open(
            RamFileSystems, FTS_PHYSICAL | FTS_XDEV, NULL)))
    {
        int total = 0;
        FTSENT *ftsent;
        while (ftsent = fts_read(fts),  ftsent != NULL)
            if (ftsent->fts_info != FTS_D)
                total += ftsent->fts_statp->st_size;
        fts_close(fts);
        return total;
    }
    else
        return 0;
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
    FILE * MemInfo;
    if (TEST_NULL(MemInfo = fopen("/proc/meminfo", "r")))
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


static void ReadTemperature(const char * sensor, int *result)
{
    /* Annoyingly the format of the temperature readout depends on which
     * system version we're using! */
    ParseFile(sensor, 1,
        UseSys ? "%d" : "%*d %*d %d", result);
    if (UseSys)
        *result /= 1000;
}

/* The second RF board sensor is read differently.  We return the result in
 * millidegrees, and the layout of the data in the /proc node is completely
 * different! */
static void ReadTemperatureRF2(const char * sensor, int *result)
{
    if (UseSys)
        ParseFile(sensor, 1, "%d", result);
    else
    {
        int degrees, millidegrees;
        ParseFile(sensor, 2, "%*s %*s %*s %d.%d", &degrees, &millidegrees);
        *result = 1000 * degrees + millidegrees;
    }
}



/* The following reads the key system health parameters directly from the
 * appropriate devices and proc/sys files. */

static void ReadHealth()
{
    if (LiberaBrilliance)
    {
        /* Only read the RF sensors if we're running Brilliance, as otherwise
         * it's disabled as it disturbs the position measurement too much. */
        ReadTemperature(proc_temp_rf1, &RfTemperature1);
        ReadTemperatureRF2(proc_temp_rf2, &RfTemperature2);
    }
    ReadTemperature(proc_temp_mb, &MbTemperature);

    ParseFile(proc_fan0,     1, "%d", &FanSpeeds[0]);
    ParseFile(proc_fan1,     1, "%d", &FanSpeeds[1]);
    ParseFile(proc_fan0_set, 1, "%d", &FanSetSpeeds[0]);
    ParseFile(proc_fan1_set, 1, "%d", &FanSetSpeeds[1]);

    /* The system voltages are read directly from the msp device in binary
     * format.  This particular step takes a surprisingly long time (about
     * half a second) -- in particular, this one step requires all our
     * processing to be done in the sensors thread (rather than the
     * alternative of using an EPICS SCAN thread). */
    int msp;
    if (TEST_IO(msp = open("/dev/msp0", O_RDONLY)))
    {
        read(msp, SystemVoltages, sizeof(SystemVoltages));
        close(msp);
    }
}



/*****************************************************************************/
/*                                                                           */
/*                           NTP Status Monitoring                           */
/*                                                                           */
/*****************************************************************************/


enum {
    NTP_NOT_MONITORED,  // Monitoring disabled (or not yet happened)
    NTP_NO_NTP,         // No NTP server running locally
    NTP_STARTUP,        // Startup grace period
    NTP_NO_SYNC,        // NTP running but not synchronised
    NTP_OK,             // NTP running ok.
};
static int NTP_status = NTP_NOT_MONITORED;
static int NTP_stratum = 16;    // 16 means unreachable/invalid server
static EPICS_STRING NTP_server;
static bool MonitorNtp; // Can be disabled


/* The NTP server can take more than 20 minutes to satisfy itself before
 * reporting synchronisation.  During this startup period we don't report an
 * error if synchronisation has not been established. */
static const int NTP_startup_window = 1500;


/* NTP/SNTP message packet (except for NTP control messages).  See RFC 1305
 * for NTP and RFC 2030 for SNTP.
 *    Note that as this packet goes over the wire, it is necessary to use
 * hton or ntoh transformations on all the fields. */

struct ntp_pkt {
    /* Bits 0-2: "mode" or message type:
     *           3 => client request, 4 => server response
     *           6 => NTP control message (different packet format)
     * Bits 3-5: NTP version number (can use 3 or 4 here)
     * Bits 6-7: Leap indicator and alarm indication (3 => unsynchronised). */
    uint8_t li_vn_mode;     // LI[7-6]:VN[5-3]:MODE[2-0]
    uint8_t stratum;        // Stratum level of clock
    int8_t ppoll;           // log_2(poll interval) in seconds
    int8_t precision;       // log_2(clock precision) in seconds
    int32_t rootdelay;      // 2^16 * Delay to reference in seconds
    int32_t rootdispersion; // 2^16 * Root dispersion in seconds
    int32_t refid;          // IP address of reference source (stratum > 1)
    uint64_t reftime;       // Time clock was last set (2^32*seconds in epoch)
    uint64_t org;           // Time this response left server
    uint64_t rec;           // Time this request received by server
    uint64_t xmt;           // Time this request left the client
};


/* This routine sends a single UDP message to the specified address and port,
 * and waits until timeout for a reply.  If a reply of some sort was received
 * this is returned with success.  Normal failure to receive a reply is
 * silent, as this is operationally normal and reported elsewhere.
 *     The timeout is in milliseconds, and cannot be more than 999. */

static bool UdpExchange(
    const char * address, int port, time_t timeout_ms,
    const void * tx_buffer, size_t tx_length,
    void * rx_buffer, size_t *rx_length)
{
    struct sockaddr_in ntp_server;
    memset(&ntp_server, 0, sizeof(ntp_server));
    ntp_server.sin_family = AF_INET;
    ntp_server.sin_port = htons(port);
    inet_aton(address, &ntp_server.sin_addr);

    int sock;
    ssize_t rx = 0;
    bool Ok = TEST_IO(sock = socket(AF_INET, SOCK_DGRAM, 0));
    if (Ok)
    {
        size_t sent;
        
        fd_set rx_ready;
        FD_ZERO(&rx_ready);
        FD_SET(sock, &rx_ready);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000 * timeout_ms;
            
        int sel;
        Ok =
            TEST_IO(connect(sock,
                (const struct sockaddr *) &ntp_server, sizeof(ntp_server)))  &&
            TEST_IO(sent = send(sock, tx_buffer, tx_length, 0))  &&
            TEST_OK(sent == tx_length)  &&
            TEST_IO(sel = select(sock+1, &rx_ready, NULL, NULL, &timeout))  &&
            /* Fail if select timed out. */
            sel > 0  &&
            /* Read can fail, and we don't actually want to log this. */
            DO_(rx = recv(sock, rx_buffer, *rx_length, 0))  &&
            rx != -1;
        
        TEST_IO(close(sock));
    }
    
    *rx_length = Ok ? rx : 0;
    return Ok;
}


/* Simply sends an SNTP packet to the given server, waits for a response or
 * timeout, and does simple validation of the response. */

static bool SNTP_exchange(
    const char * address, time_t timeout_ms, ntp_pkt *result)
{
    /* Might as well use the result packet for our transmit.  For a simple
     * SNTP status request we can just set the whole packet to zero (except
     * for the mode byte). */
    memset(result, 0, sizeof(ntp_pkt));
    result->li_vn_mode = (0 << 6) | (3 << 3) | (3 << 0);
    size_t rx = sizeof(ntp_pkt);
    return
        UdpExchange(address, 123, timeout_ms, result, rx, result, &rx)  &&
        /* Simple validation. */
        rx == sizeof(ntp_pkt)  &&       // Complete packet received
        (result->li_vn_mode & 7) == 4;  // Response is server mode response
}


/* For high stratum values the refid is the IP address of the reference
 * server, for stratum values 0 and 1 the refid is a four character string. */

static void refid_to_string(int stratum, int refid, EPICS_STRING string)
{
#define ID_BYTE(i) ((refid >> (8*(i))) & 0xFF)
    if (stratum > 1)
        snprintf(string, sizeof(EPICS_STRING),
            "%d.%d.%d.%d", ID_BYTE(0), ID_BYTE(1), ID_BYTE(2), ID_BYTE(3));
    else
    {
        memcpy(string, &refid, sizeof(refid));
        string[4] = 0;
    }
#undef ID_BYTE
}


static void ProcessNtpHealth()
{
    ntp_pkt pkt;
    if (SNTP_exchange("127.0.0.1", 100, &pkt))
    {
        int LI = (pkt.li_vn_mode >> 6) & 3;
        NTP_status = LI == 3 ?
            (LastUptime < NTP_startup_window ?
                NTP_STARTUP : NTP_NO_SYNC) : NTP_OK;
        NTP_stratum = pkt.stratum == 0 ? 16 : pkt.stratum;
        refid_to_string(pkt.stratum, pkt.refid, NTP_server);
    }
    else
    {
        NTP_status = NTP_NO_NTP;
        NTP_stratum = 16;
        strcpy(NTP_server, "no server");
    }
}




/*****************************************************************************/
/*                                                                           */
/*                           Sensors Initialisation                          */
/*                                                                           */
/*****************************************************************************/



static void ProcessSensors()
{
    ProcessUptimeAndIdle();
    ProcessFreeMemory();
    if (EnableSensors)
        ReadHealth();
    if (MonitorNtp)
        ProcessNtpHealth();
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


static void SendHealthCommand(const char * command, ...)
    __attribute__((format(printf, 1, 2)));
static void SendHealthCommand(const char * command, ...)
{
    FILE * fifo;
    if (TEST_NULL(fifo = fopen(HEALTHD_COMMAND_FIFO, "w")))
    {
        va_list args;
        va_start(args, command);
        vfprintf(fifo, command, args);
        fclose(fifo);
    }
}


static void SetEnableSensors()
{
    /* Ensure the state of the health daemon is in step. */
    SendHealthCommand(EnableSensors ? "ON\n" : "OFF\n");
}


static void SetTargetTemperature()
{
    SendHealthCommand("T%d\n", TargetTemperature);
}


#define PUBLISH_BLOCK(Type, BaseName, Array) \
    ( { \
        for (unsigned int i = 0; i < ARRAY_SIZE(Array); i ++) \
        { \
            char * Name = (char *) malloc(strlen(BaseName) + 5); \
            sprintf(Name, BaseName, i + 1); \
            Publish_##Type(Name, Array[i]); \
        } \
    } )


#define I2C_DEVICE  "/sys/bus/i2c/devices/"
#define PROC_DEVICE "/proc/sys/dev/sensors/"

bool InitialiseSensors(bool _MonitorNtp)
{
    MonitorNtp = _MonitorNtp;
    
    /* Figure out where to read our fan and temperature sensors: under Linux
     * 2.6 we read from the /sys file system, but under 2.4 we read from /proc
     * instead. */
    UseSys = access("/sys", F_OK) == 0;
    if (UseSys)
    {
        /* The /sys file system exists.  All our sensors live here. */
        proc_temp_rf1 = I2C_DEVICE "0-0018/temp1_input";
        proc_temp_rf2 = I2C_DEVICE "0-0018/temp2_input";
        proc_temp_mb  = I2C_DEVICE "0-0029/temp1_input";
        proc_fan0     = I2C_DEVICE "0-004b/fan1_input";
        proc_fan1     = I2C_DEVICE "0-0048/fan1_input";
        /* The fan speed control is different depending on the kernel
         * version, alas.  We try for the newer version first, dropping back
         * to the older version if not found. */
        proc_fan0_set = I2C_DEVICE "0-004b/fan1_target";
        proc_fan1_set = I2C_DEVICE "0-0048/fan1_target";
        if (access(proc_fan0_set, F_OK) != 0)
        {
            proc_fan0_set = I2C_DEVICE "0-004b/speed";
            proc_fan1_set = I2C_DEVICE "0-0048/speed";
        }
    }
    else
    {
        /* No /sys file system: revert to the older /proc filesystem. */
        proc_temp_rf1 = PROC_DEVICE "adm1023-i2c-0-18/temp1";
        proc_temp_rf2 = PROC_DEVICE "adm1023-i2c-0-18/temp2";
        proc_temp_mb  = PROC_DEVICE "max1617a-i2c-0-29/temp1";
        proc_fan0     = PROC_DEVICE "max6650-i2c-0-4b/fan1";
        proc_fan1     = PROC_DEVICE "max6650-i2c-0-48/fan1";
        proc_fan0_set = PROC_DEVICE "max6650-i2c-0-4b/speed";
        proc_fan1_set = PROC_DEVICE "max6650-i2c-0-48/speed";
    }

    Publish_longin("SE:TEMP",
        LiberaBrilliance ? RfTemperature1 : MbTemperature);
    Publish_longin("SE:TEMP_RF1",  RfTemperature1);
    Publish_ai    ("SE:TEMP_RF2",  RfTemperature2);
    Publish_longin("SE:TEMP_MB",   MbTemperature);
    PUBLISH_BLOCK(longin, "SE:FAN%d",     FanSpeeds);
    PUBLISH_BLOCK(longin, "SE:FAN%d_SET", FanSetSpeeds);
    PUBLISH_BLOCK(ai,     "SE:VOLT%d",    SystemVoltages);

    Publish_ai("SE:FREE",    MemoryFree);
    Publish_ai("SE:RAMFS",   RamfsUsage);
    Publish_ai("SE:UPTIME",  Uptime);
    Publish_ai("SE:EPICSUP", EpicsUp);
    Publish_ai("SE:CPU",     CpuUsage);

    PUBLISH_FUNCTION_OUT(bo, "SE:ENABLE", EnableSensors, SetEnableSensors);
    PUBLISH_CONFIGURATION(longout, "SE:SETTEMP",
        TargetTemperature, SetTargetTemperature);

    /* Although these are processed here as sensors, these fields are
     * aggregated as part of the clock subsystem. */
    Publish_mbbi("CK:NTPSTAT", NTP_status);
    Publish_longin("CK:STRATUM", NTP_stratum);
    Publish_stringin("CK:SERVER", NTP_server);

    InitialiseUptime();
    SetEnableSensors();
    SetTargetTemperature();

    SensorsThread = new SENSORS_THREAD();
    return
        InitialiseRamfsUsage()  &&
        SensorsThread->StartThread();
}



void TerminateSensors()
{
    if (SensorsThread != NULL)
        SensorsThread->Terminate();
}
