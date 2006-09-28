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


/* Simple wrapper for Linux based ioc.  This is the command line wrapper
 * for a standalone daemon ioc. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <string.h>

#include <iocsh.h>

#include "firstTurn.h"
#include "booster.h"
#include "turnByTurn.h"
#include "freeRun.h"
#include "slowAcquisition.h"
#include "postmortem.h"
#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "thread.h"
#include "trigger.h"
#include "interlock.h"
#include "sensors.h"

#include "fastFeedback.h"

#include "events.h"
#include "hardware.h"
#include "convert.h"


/* This variable records the PID file: if successfully written then it will
 * be removed when terminated. */
static const char * PidFileName = NULL;

/* This controls whether an IOC shell is used.  If not the main thread will
 * sleep indefinitely, waiting for the IOC to be killed by a signal. */
static bool RunIocShell = true;

/* If the IOC shell is not running then this semaphore is used to request IOC
 * shutdown. */
static sem_t ShutdownSemaphore;


/* Configuration settable parameters.
 *
 * Note that although defaults have been defined for all of these values, all
 * of these parameters are normally passed in by the runioc script. */

/* Maximum length of long turn by turn buffer. */
static int LongTurnByTurnLength = 196608;       // 12 * default window length
static int TurnByTurnWindowLength = 16384;
/* Free running window length. */
static int FreeRunLength = 2048;
/* Length of 1024 decimated buffer. */
static int DecimatedShortLength = 190;

/* Synchrotron revolution frequency.  Used for labelling decimated data.
 * This default frequency is the Diamond booster frequency. */
static float RevolutionFrequency = 1892629.155;

/* Fundamental ring parameters.  The defaults are for the Diamond storage
 * ring. */
static int Harmonic = 936;              // Bunches per revolution
static int Decimation = 220;            // Samples per revolution
static int LmtdPrescale = 53382;        // Prescale for lmtd

/* Location of the persistent state file. */
static const char * StateFileName = NULL;


static EPICS_STRING VersionString = LIBERA_VERSION;
static EPICS_STRING BuildDate = BUILD_DATE_TIME;


/* Prints interactive startup message as recommended by GPL. */

static void StartupMessage()
{
    printf(
"\n"
"Libera EPICS Driver, Version %s.  Built: %s.\n"
"\n"
"Copyright (C) 2005-2006 Michael Abbott, Diamond Light Source.\n"
"This program comes with ABSOLUTELY NO WARRANTY.  This is free software,\n"
"and you are welcome to redistribute it under certain conditions.\n"
"For details see the GPL or the attached file COPYING.\n",
        VersionString, BuildDate);
}



/* We take any of the four traditional shutdown signals (HUP, INT, QUIT or
 * TERM) as a request to terminate the IOC.  We let the IOC terminate in an
 * orderly manner, so the task is to interrupt whatever the main ioc loop is
 * up to.
 *     Note that this code is in a signal handler, so we must take care to
 * only call async-safe functions.  This rather restricts what we're allowed
 * to do!
 *     Note also that this handler can be called repeatedly on exit (I'm not
 * sure why: perhaps it gets called on each running thread?) so its actions
 * need to be idempotent.  Fortunately close doesn't seem to mind being called
 * repeatedly, and multiple posts to our shutdown semaphore are entirely
 * inconsequential. */

static void AtExit(int signal)
{
    if (RunIocShell)
        /* If the IOC shell is running then closing stdin is sufficient to
         * cause iocsh() to terminate. */
        close(0);
    else
        /* If the IOC shell is not running then the main thread is waiting
         * for our sem_post. */
        sem_post(&ShutdownSemaphore);
}

static bool InitialiseAtExit()
{
    struct sigaction action;
    action.sa_handler = AtExit;
    action.sa_flags = 0;
    return
        TEST_(sigfillset, &action.sa_mask)  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_(sigaction, SIGHUP,  &action, NULL)  &&
        TEST_(sigaction, SIGINT,  &action, NULL)  &&
        TEST_(sigaction, SIGQUIT, &action, NULL)  &&
        TEST_(sigaction, SIGTERM, &action, NULL);
}


/* Configures non-interactive (daemon mode) operation. */

static void SetNonInteractive()
{
    close(0);
    RunIocShell = false;
    sem_init(&ShutdownSemaphore, 0, 0);
}



/* Prints usage message in response to -h option. */

static void Usage(const char *IocName)
{
    printf(
"Usage: %s [-p <pid-file>] <scripts>\n"
"Runs Libera EPICS ioc with an interactive IOC shell after loading and\n"
"running <scripts> as IOC scripts.\n"
"\n"
"Options:\n"
"    -h                 Writes out this usage description.\n"
"    -p <pid-file>      Writes pid to <pid-file>.\n"
"    -n                 Run non-interactively without an IOC shell\n"
"    -v                 Writes version information\n"
"    -c<key>=<value>    Configure run time parameter.  <key> can be:\n"
"       LT      Length of long turn-by-turn buffer\n"
"       TT      Length of short turn-by-turn buffer\n"
"       TW      Length of turn-by-turn readout window\n"
"       DD      Length of /1024 decimated data buffer\n"
"       HA      Harmonic: number of bunches per revolution\n"
"       DE      Decimation: number of samples per revolution\n"
"       LP      LMTD prescale factor\n"
"    -s <state-file>    Read and record persistent state in <state-file>\n"
"\n"
"Note: This IOC application should normally be run from within runioc.\n",
        IocName);
}


/* The Libera driver is started by starting all of its constituent components
 * in turn.  Here is the natural place for these to be defined. */

static bool InitialiseLibera()
{
    return
        /* Set up exit hander. */
        InitialiseAtExit()  &&

        /* Initialise the persistent state system early on so that other
         * components can make use of it. */
        InitialisePersistentState(StateFileName)  &&
        /* Initialise the connections to the Libera device. */
        InitialiseHardware()  &&
        /* Initialise conversion code.  This needs to be done fairly early as
         * it is used globally. */
        InitialiseConvert(LmtdPrescale, Decimation, Harmonic)  &&
        /* Get the event receiver up and running.  This spawns a background
         * thread for dispatching trigger events. */
        InitialiseEventReceiver()  &&
        /* Ensure the trigger interlock mechanism is working. */
        InitialiseTriggers()  &&

        /* Now we can initialise the mode specific components. */

        /* Initialise interlock settings. */
        InitialiseInterlock()  &&
        /* First turn processing is designed for transfer path operation. */
        InitialiseFirstTurn(Harmonic, Decimation)  &&
        /* Turn by turn is designed for long waveform capture at revolution
         * clock frequencies. */
        InitialiseTurnByTurn(LongTurnByTurnLength, TurnByTurnWindowLength)  &&
        /* Free run also captures turn by turn waveforms, but of a shorter
         * length that can be captured continously. */
        InitialiseFreeRun(FreeRunLength)  &&
        /* Booster operation is designed for viewing the entire booster ramp
         * at reduced resolution. */
        InitialiseBooster(DecimatedShortLength, RevolutionFrequency)  &&
        /* Postmortem operation is only triggered on a postmortem event and
         * captures the last 16K events before the event. */
        InitialisePostmortem()  &&
        /* Slow acquisition returns highly filtered positions at 10Hz. */
        InitialiseSlowAcquisition()  &&

#ifdef BUILD_FF_SUPPORT
        /* Initialise the fast feedback interface. */
        InitialiseFastFeedback()  &&
#endif

        /* Background monitoring stuff: fan, temperature, memory, etcetera. */
        InitialiseSensors();
}


/* Shutting down is really just a matter of closing down the event receiver
 * (because it runs a separate thread: this needs to be done in an orderly
 * way, otherwise we'll crash) and closing our connections to the Libera
 * driver (to be tidy). */

static void TerminateLibera()
{
    TerminateEventReceiver();
    TerminateSlowAcquisition();
    TerminateHardware();
    TerminatePersistentState();
    
#ifdef BUILD_FF_SUPPORT
    TerminateFastFeedback();
#endif
        
    /* On orderly shutdown remove the pid file if we created it.  Do this
     * last of all. */
    if (PidFileName != NULL)
        unlink(PidFileName);
}



/* Write the PID of this process to the given file. */

static bool WritePid(const char * FileName)
{
    FILE * output = fopen(FileName, "w");
    if (output == NULL)
    {
        perror("Can't open PID file");
        return false;
    }
    else
    {
        /* Lazy error checking here.  Really should check that there aren't
         * any errors in any of the following. */
        fprintf(output, "%d", getpid());
        fclose(output);
        /* Remember PID filename so we can remove it on exit. */
        PidFileName = FileName;
        return true;
    }
}


/* This routine parses a configuration setting.  This will be of the form
 *
 *      <key>=<value>
 *
 * where <key> identifies which value is set and <value> is an integer. */

static bool ParseConfigInt(char *optarg)
{
    static const struct
    {
        const char * Name;
        int & Target;
    } Lookup[] = {
        { "TT", LongTurnByTurnLength },
        { "TW", TurnByTurnWindowLength },
        { "FR", FreeRunLength },
        { "BN", DecimatedShortLength },
        { "HA", Harmonic },
        { "DE", Decimation },
        { "LP", LmtdPrescale },
    };

    /* Parse the configuration setting into <key>=<integer>. */
    char * eq = strchr(optarg, '=');
    if (eq == NULL)
    {
        printf("Ill formed config definition: \"%s\"\n", optarg);
        return false;
    }
    *eq++ = '\0';
    char * end;
    int Value = strtol(eq, &end, 0);
    if (eq == end  ||  *end != '\0')
    {
        printf("Configuration value not a number: \"%s=%s\"\n", optarg, eq);
        return false;
    }
    
    /* Figure out who it belongs to! */
    for (size_t i = 0; i < ARRAY_SIZE(Lookup); i ++)
    {
        if (strcmp(optarg, Lookup[i].Name) == 0)
        {
            Lookup[i].Target = Value;
            return true;
        }
    }

    /* Nope, never heard of it. */
    printf("Unknown configuration value \"%s\"\n", optarg);
    return false;
}


/* Parses a floating point number, reports if invalid. */

static bool ParseFloat(const char *optarg, float &Target)
{
    char *end;
    Target = strtod(optarg, &end);
    if (optarg == end  ||  *end != '\0')
    {
        printf("Not a floating point number: \"%s\"\n", optarg);
        return false;
    }
    else
        return true;
}


/* Process any options supported by the ioc.  At present we support:
 *
 *      -h              Print out usage
 *      -p<filename>    Write ioc PID to <filename>
 *
 * argc and argv are updated to point past the options. */

static bool ProcessOptions(int &argc, char ** &argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(argc, argv, "+p:nc:f:s:hv"))
        {
            case 'p':   Ok = WritePid(optarg);          break;
            case 'n':   SetNonInteractive();            break;
            case 'c':   Ok = ParseConfigInt(optarg);    break;
            case 'f':   Ok = ParseFloat(
                            optarg, RevolutionFrequency);  break;
            case 's':   StateFileName = optarg;         break;
            case 'h':   Usage(argv[0]);                 return false;
            case 'v':   StartupMessage();               return false;
            case '?':
            default:
                printf("Try `%s -h` for usage\n", argv[0]);
                return false;
            case -1:
                /* All arguments read successfuly.  Consume them and return
                 * success. */
                argc -= optind;
                argv += optind;
                return true;
        }
    }
    /* Drop through to here if processing an option fails. */
    return false;
}


int main(int argc, char *argv[])
{
    Publish_stringin("VERSION", VersionString);
    Publish_stringin("BUILD", BuildDate);
    
    /* Consume any option arguments and start the driver. */
    bool Ok =
        ProcessOptions(argc, argv)  &&
        InitialiseLibera();

    /* Consume any remaining script arguments by running them through the IOC
     * shell. */
    for (; Ok  &&  argc > 0; argc--)
        Ok = iocsh(*argv++) == 0;

    /* Run the entire IOC with a live IOC shell, or just block with the IOC
     * running in the background. */
    if (Ok)
    {
        StartupMessage();
        if (RunIocShell)
            /* Run an interactive shell. */
            Ok = iocsh(NULL);
        else
        {
            /* Wait for the shutdown request. */
            printf("Running in non-interactive mode.  "
                "Kill process %d to close.\n", getpid());
            fflush(stdout);
            sem_wait(&ShutdownSemaphore);
        }
    }

    /* Finally shut down in as tidy a manner as possible.  Note that this
     * routine may be called without the matching InitialiseLibera() routine
     * being called: all the terminate routines need to handle this
     * possibility cleanly. */
    TerminateLibera();
    printf("Ioc terminated normally\n");
    return Ok ? 0 : 1;
}
