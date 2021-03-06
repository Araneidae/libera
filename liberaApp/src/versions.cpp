/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009-2011  Michael Abbott, Diamond Light Source Ltd.
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

/* System version identification PVs. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <limits.h>
#include <fcntl.h>

#include <epicsVersion.h>

#include "device.h"
#include "publish.h"
#include "hardware.h"

#include "versions.h"




static EPICS_STRING VersionString = LIBERA_VERSION;
static EPICS_STRING BuildDate = BUILD_DATE_TIME;
static EPICS_STRING EpicsVersion = EPICS_VERSION_STRING;

int DecimationFactor;
int FA_FIR_Decimation;
int FA_DecimationFirLength;
bool LiberaBrilliance;
bool OldBrillianceApi;
bool FastFeedbackFeature;
bool DlsFpgaFeatures;
bool MafFeaturePresent;
bool ItechMaxAdcPresent;
bool Version2FpgaPresent;
bool SecondaryInterlock;
bool FAPayloadSelection;
#ifdef __EBPP_H_2
bool Version2Driver = true;
#else
bool Version2Driver = false;
#endif

static int CustomerId;
static EPICS_STRING CustomerIdString;
static EPICS_STRING CompilerVersion;
static EPICS_STRING LibraryVersion;



/*****************************************************************************/
/*                                                                           */
/*                        Reboot and Restart Support                         */
/*                                                                           */
/*****************************************************************************/


/* This is a fairly dirty system for ensuring that we can restart either EPICS
 * or the entire Libera even when fairly low on memory. */

static void DetachProcess(const char *Process, const char *const argv[])
{
    /* Annoyingly we need to fork a new process because the first thing that
     * `/etc/init.d/epics restart` does is to kill the old PID, so we need a new
     * one.  We need to use vfork() here because if we are low on memory, for
     * example if a full length TT waveform has been fetched, then fork() will
     * fail. */
    pid_t pid;
    if (TEST_IO(pid = vfork()))
    {
        if (pid == 0)
        {
            /* We're not obeying the rules for forkv(), but we should get away
             * with it.  All the calls we're making are system calls which
             * should only affect the new process, and the old one is going to
             * be gone soon anyway. */

            /* Ensure that none of our open files will be inherited.  It's safer
             * to do this than to close them. */
            for (int i = 3; i < sysconf(_SC_OPEN_MAX); i ++)
                fcntl(i, F_SETFD, FD_CLOEXEC);

            /* Enable all signals. */
            sigset_t sigset;
            sigfillset(&sigset);
            sigprocmask(SIG_UNBLOCK, &sigset, NULL);

            /* Finally we can actually exec the new process... */
            char * envp[] = { NULL };
            execve(Process, (char**) argv, envp);
        }
    }
}


static void DoReboot()
{
    printf("Libera reboot requested\n");
    const char * Args[] = { "/sbin/reboot", NULL };
    DetachProcess(Args[0], Args);
}


static void DoRestart()
{
    printf("EPICS IOC restart requested\n");
    const char * Args[] = { "/etc/init.d/epics", "restart", NULL };
    DetachProcess(Args[0], Args);
}



/*****************************************************************************/
/*                                                                           */
/*                          Version Identification                           */
/*                                                                           */
/*****************************************************************************/


template<class T>
struct ENV_MAP
{
    const char * env_name;
    const char * pv_name;
    T *value;
};


static ENV_MAP<EPICS_STRING> EnvironmentStrings[] = {
    { "ABI_VERSION",    "ABI" },
    { "KERNEL_VERSION", "UNAME" },
    { "LIBC_VERSION",   "LIBC" },
    { "LIBERA_VERSION", "DRIVER" },
    { "MSP_VERSION",    "MSP" },
    { "ROOTFS_ARCH",    "ARCH" },
    { "ROOTFS_VERSION", "ROOTFS" },
    { "FPGA_VERSION",   "FPGA" },
};

static ENV_MAP<int> EnvironmentInts[] = {
    { "FPGA_COMPILED",  "COMPILED" },
    { "FPGA_BUILD_NO",  "BUILDNO" },
    { "FPGA_CUST_ID",   "CUSTID",   &CustomerId },
    { "FPGA_DDC_DEC",   "DDCDEC",   &DecimationFactor },
    { "FPGA_FA_CIC",    "FACIC" },
    { "FPGA_FA_FIR",    "FAFIR",    &FA_FIR_Decimation },
    { "FPGA_FA_DEC",    "FADEC" },
    { "FPGA_CUSTOMER",  "CUSTOMER" },
    { "FPGA_ITECH",     "ITECH" },
};

static ENV_MAP<bool> EnvironmentBools[] = {
    { "OPT_BR",         "BR",       &LiberaBrilliance },
    { "BR_TEMP",        "BRHW" },
    { "OPT_OLD_BR",     "OLDBR",    &OldBrillianceApi },
    { "OPT_DLS_FPGA",   "DLS",      &DlsFpgaFeatures },
    { "OPT_FF",         "FF",       &FastFeedbackFeature },
    { "OPT_GBETH",      "GBETH" },
    { "OPT_MAF",        "MAF",      &MafFeaturePresent },
    { "ITECH_MAX_ADC",  "ITMAXADC", &ItechMaxAdcPresent },
    { "FPGA_2_SUPPORT", "FPGA2",    &Version2FpgaPresent },
    { "SECONDARY_ILK",  "ILK2",     &SecondaryInterlock },
    { "FA_PAYLOAD_SEL", "FAPAY",    &FAPayloadSelection },
};


template<class T>
static bool PublishEnvMap(
    ENV_MAP<T> env_map[], unsigned int count,
    bool (*convert)(const char * string, T &value),
    void (*publish)(const char * Name, T &value))
{
    for (unsigned int i = 0; i < count; i ++)
    {
        ENV_MAP<T> &map = env_map[i];
        char * string = getenv(map.env_name);
        if (string == NULL)
        {
            printf("Unable to read environment variable %s\n", map.env_name);
            return false;
        }
        if (map.value == NULL)
            map.value = (T *) malloc(sizeof(T));
        if (! convert(string, *map.value))
        {
            printf("Error converting %s=\"%s\"\n", map.env_name, string);
            return false;
        }
        publish(Concat("VE:", map.pv_name), *map.value);
    }
    return true;
}

#define PUBLISH_ENV_MAP(record, map) \
    PublishEnvMap<TYPEOF(record)>( \
        map, ARRAY_SIZE(map), convert_##record, Publish_##record)

static bool convert_stringin(const char * string, EPICS_STRING &value)
{
    strncpy(value, string, sizeof(value));
    value[sizeof(value)-1] = 0;
    return true;
}

static bool convert_longin(const char * string, int &value)
{
    char *end;
    value = strtol(string, &end, 10);
    return string < end  &&  *end == '\0';
}

static bool convert_bi(const char * string, bool &value)
{
    value = string[0] == '1';
    return (string[0] == '0' || string[0] == '1')  &&  string[1] == '\0';
}


/* Converts the customer ID to a string.  For whatever reason, it is stored
 * in big endian order, so we need to reverse it.  Also we skip any leading
 * bytes if necessary. */
static void IdToString(int Id, char *string)
{
#define ID_CHAR(i)     (Id >> (24 - 8 * (i)))
    int i = 0;
    while (i < 4  &&  ID_CHAR(i) == 0)
        i ++;
    for ( ; i < 4; i ++)
        *string++ = ID_CHAR(i);
#undef ID_CHAR
}




bool InitialiseVersions(void)
{
    /* This code here which tries to work out which versions of the compiler
     * and library we're being compiled with uses the very helpful pages at
     *      http://predef.sourceforge.net
     * We do our best to figure out values to assign our version strings.
     * Unfortunately, the identification strings are compiler specific, so
     * here we can only support the compilers and libraries we know about. */
    snprintf(CompilerVersion, sizeof(EPICS_STRING),
#if defined(__GNUC__)
        "gcc %d.%d.%d", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
        "unknown compiler");
#endif
    snprintf(LibraryVersion, sizeof(EPICS_STRING),
#if defined(__GLIBC__)
        "glibc %d.%d", __GLIBC__, __GLIBC_MINOR__);
#else
        "unknown library");
#endif

    Publish_stringin("VERSION",     VersionString);
    Publish_stringin("BUILD",       BuildDate);

    Publish_stringin("VE:VERSION",  VersionString);
    Publish_stringin("VE:BUILD",    BuildDate);
    Publish_stringin("VE:EPICS",    EpicsVersion);
    Publish_stringin("VE:CUSTIDSTR", CustomerIdString);
    Publish_stringin("VE:COMPILER", CompilerVersion);
    Publish_stringin("VE:LIBRARY",  LibraryVersion);
    Publish_bi      ("VE:DRIVER2",  Version2Driver);

    PUBLISH_ACTION("REBOOT",        DoReboot);
    PUBLISH_ACTION("RESTART",       DoRestart);

    bool Ok =
        PUBLISH_ENV_MAP(stringin,   EnvironmentStrings)  &&
        PUBLISH_ENV_MAP(longin,     EnvironmentInts)  &&
        PUBLISH_ENV_MAP(bi,         EnvironmentBools);
    /* Must do this after the above so that CustomerId is initialised. */
    IdToString(CustomerId, CustomerIdString);
    /* The FA decimation FIR length is hard-wired in the FPGA to be three times
     * the decimation factor, so we compute this here.  This knowledge is also
     * hard-wired into hardware.cpp. */
    FA_DecimationFirLength = 3 * FA_FIR_Decimation;

    return Ok;
}



/* Prints interactive startup message as recommended by GPL. */

void StartupMessage()
{
    printf(
"\n"
"Libera EPICS Driver, Version %s.  Built: %s.\n"
"Compiled with %s, linked with %s\n"
"\n"
"Copyright (C) 2005-2011 Michael Abbott, Diamond Light Source.\n"
"This program comes with ABSOLUTELY NO WARRANTY.  This is free software,\n"
"and you are welcome to redistribute it under certain conditions.\n"
"For details see the GPL or the attached file COPYING.\n",
        VersionString, BuildDate, CompilerVersion, LibraryVersion);
}
