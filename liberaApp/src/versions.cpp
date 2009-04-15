/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009  Michael Abbott, Diamond Light Source Ltd.
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
#include <signal.h>
#include <string.h>
#include <sys/wait.h>
#include <arpa/inet.h>

#include <epicsVersion.h>

#include "test_error.h"
#include "device.h"
#include "publish.h"

#include "versions.h"




static EPICS_STRING VersionString = LIBERA_VERSION;
static EPICS_STRING BuildDate = BUILD_DATE_TIME;
static EPICS_STRING EpicsVersion = EPICS_VERSION_STRING;

int DecimationFactor;
bool LiberaBrilliance;
bool BrillianceInverted;
bool FastFeedbackFeature;
bool DlsFpgaFeatures;

static int CustomerId;
static EPICS_STRING CustomerIdString;



/*****************************************************************************/
/*                                                                           */
/*                        Reboot and Restart Support                         */
/*                                                                           */
/*****************************************************************************/


/* Fairly generic routine to start a new detached process in a clean
 * environment. */

static void DetachProcess(const char *Process, const char *const argv[])
{
    /* We fork twice to avoid leaving "zombie" processes behind.  These are
     * harmless enough, but annoying.  The double-fork is a simple enough
     * trick. */
    pid_t MiddlePid = fork();
    if (MiddlePid == -1)
        perror("Unable to fork");
    else if (MiddlePid == 0)
    {
        pid_t NewPid = fork();
        if (NewPid == -1)
            perror("Unable to fork");
        else if (NewPid == 0)
        {
            /* This is the new doubly forked process.  We still need to make
             * an effort to clean up the environment before letting the new
             * image have it. */

            /* Set a sensible home directory. */
            chdir("/");

            /* Enable all signals. */
            sigset_t sigset;
            sigfillset(&sigset);
            sigprocmask(SIG_UNBLOCK, &sigset, NULL);

            /* Close all the open file handles.  It's rather annoying: there
             * seems to be no good way to do this in general.  Fortunately in
             * our case _SC_OPEN_MAX is managably small. */
            for (int i = 0; i < sysconf(_SC_OPEN_MAX); i ++)
                close(i);

            /* Finally we can actually exec the new process... */
            char * envp[] = { NULL };
            execve(Process, (char**) argv, envp);
        }
        else
            /* The middle process simply exits without further ceremony.  The
             * idea here is that this orphans the new process, which means
             * that the parent process doesn't have to wait() for it, and so
             * it won't generate a zombie when it exits. */
            _exit(0);
    }
    else
        /* Wait for the middle process to finish. */
        TEST_(waitpid, MiddlePid, NULL, 0);
}


static void DoReboot()
{
    const char * Args[] = { "/sbin/reboot", NULL };
    DetachProcess(Args[0], Args);
}


static void DoRestart()
{
    const char * Args[] = { "/etc/init.d/epics", "restart", NULL };
    DetachProcess(Args[0], Args);
}


static void DoCoreDump()
{
    * (char *) 0 = 0;
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
};

static ENV_MAP<int> EnvironmentInts[] = {
    { "FPGA_COMPILED",  "COMPILED" },
    { "FPGA_BUILD_NO",  "BUILDNO" },
    { "FPGA_CUST_ID",   "CUSTID",   &CustomerId },
    { "FPGA_DDC_DEC",   "DDCDEC",   &DecimationFactor },
    { "FPGA_FA_DEC",    "FADEC" },
    { "FPGA_CUSTOMER",  "CUSTOMER" },
    { "FPGA_ITECH",     "ITECH" },
};

static ENV_MAP<bool> EnvironmentBools[] = {
    { "OPT_BR",         "BR",       &LiberaBrilliance },
    { "OPT_BR_INVERT",  "BRINVERT", &BrillianceInverted },
    { "OPT_DLS_FPGA",   "DLS",      &DlsFpgaFeatures },
    { "OPT_FF",         "FF",       &FastFeedbackFeature },
    { "OPT_MAF",        "MAF" },
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
    value = strtol(string, &end, 0);
    return string < end  &&  *end == '\0';
}

static bool convert_bi(const char * string, bool &value)
{
    value = string[0] == '1';
    return (string[0] == '0' || string[0] == '1')  &&  string[1] == '\0';
}


bool InitialiseVersions(void)
{
    Publish_stringin("VERSION",     VersionString);
    Publish_stringin("BUILD",       BuildDate);
    Publish_stringin("VE:VERSION",  VersionString);
    Publish_stringin("VE:BUILD",    BuildDate);
    Publish_stringin("VE:EPICS",    EpicsVersion);
    Publish_stringin("VE:CUSTIDSTR", CustomerIdString);
    
    PUBLISH_ACTION("REBOOT",     DoReboot);
    PUBLISH_ACTION("RESTART",    DoRestart);
    PUBLISH_ACTION("CORE",       DoCoreDump);
    
    bool Ok =
        PUBLISH_ENV_MAP(stringin,   EnvironmentStrings)  &&
        PUBLISH_ENV_MAP(longin,     EnvironmentInts)  &&
        PUBLISH_ENV_MAP(bi,         EnvironmentBools);

    /* Convert the customer id into a string.  For whatever reason, it is
     * stored in big endian order.  As we are running little endian we can
     * use a little hack to to turn things around. */
    * (int *) CustomerIdString = ntohl(CustomerId);
    CustomerIdString[4] = 0;

    return Ok;
}



/* Prints interactive startup message as recommended by GPL. */

void StartupMessage()
{
    printf(
"\n"
"Libera EPICS Driver, Version %s.  Built: %s.\n"
"Copyright (C) 2005-2009 Michael Abbott, Diamond Light Source.\n"
"This program comes with ABSOLUTELY NO WARRANTY.  This is free software,\n"
"and you are welcome to redistribute it under certain conditions.\n"
"For details see the GPL or the attached file COPYING.\n",
        VersionString, BuildDate);
}
