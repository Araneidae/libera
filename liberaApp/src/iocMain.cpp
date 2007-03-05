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

/* IOC startup, command line processing, initialisation and shutdown. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <string.h>
#include <sys/wait.h>

#include <iocsh.h>
#include <epicsThread.h>

#include "eventd.h"     // for LIBERA_SIGNAL
#include "hardware.h"
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
#include "convert.h"
#include "configure.h"
#include "timestamps.h"


/* External declaration of caRepeater thread.  This should really be
 * published by a standard EPICS header file, but for the time being we pick
 * it up like this. */
extern "C" void caRepeaterThread(void *);



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
 * ring, but these are always overwritten when called from runioc. */
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
"Copyright (C) 2005-2007 Michael Abbott, Diamond Light Source.\n"
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


/* A quiet signal for general use.  We'll bind this to SIGUSR2. */

static void DoNothing(int signal)
{
}


/* Set up basic signal handling environment.  We configure four shutdown
 * signals (HUP, INT, QUIT and TERM) to call AtExit() and block the
 * LIBERA_SIGNAL (SIGUSR1) -- we'll unmask it where we want to catch it! */

static bool InitialiseSignals()
{
    struct sigaction AtExitHandler;
    AtExitHandler.sa_handler = AtExit;
    AtExitHandler.sa_flags = 0;
    struct sigaction DoNothingHandler;
    DoNothingHandler.sa_handler = DoNothing;
    DoNothingHandler.sa_flags = 0;
    sigset_t BlockSet;
    return
        /* Block all signals during AtExit() signal processing. */
        TEST_(sigfillset, &AtExitHandler.sa_mask)  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_(sigaction, SIGHUP,  &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGINT,  &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGQUIT, &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGTERM, &AtExitHandler, NULL)  &&

        /* Configure SIGUSR2 to do nothing: we can then use this generally
         * without side effects. */
        TEST_(sigfillset, &DoNothingHandler.sa_mask)  &&
        TEST_(sigaction, SIGUSR2, &DoNothingHandler, NULL)  &&

        /* Block the LIBERA_SIGNAL signal globally: we only want this to be
         * delivered where we're ready for it! */
        TEST_(sigemptyset, &BlockSet)  &&
        TEST_(sigaddset, &BlockSet, LIBERA_SIGNAL) &&
        TEST_(sigprocmask, SIG_BLOCK, &BlockSet, NULL);
}


/* Configures non-interactive (daemon mode) operation. */

static void SetNonInteractive()
{
    close(0);
    RunIocShell = false;
    sem_init(&ShutdownSemaphore, 0, 0);
}



/* This routine spawns a caRepeater thread, as recommended by Andrew Johnson
 * (private communication, 2006/12/04).  This means that this IOC has no
 * external EPICS dependencies (otherwise the caRepeater application needs to
 * be run). */

static bool StartCaRepeater()
{
    epicsThreadId caRepeaterId = epicsThreadCreate(
        "CAC-repeater", epicsThreadPriorityLow,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        caRepeaterThread, 0);
    if (caRepeaterId == 0)
        perror("Error starting caRepeater thread");
    return caRepeaterId != 0;
}


/* The Libera driver is started by starting all of its constituent components
 * in turn.  Here is the natural place for these to be defined. */

static bool InitialiseLibera()
{
    return
        /* Initialise signal handling.  This should happen as early as
         * possible, to ensure that all subsequent processing uses this
         * signal state. */
        InitialiseSignals()  &&
        /* First EPICS related activity. */
        StartCaRepeater()  &&

        /* Initialise the persistent state system early on so that other
         * components can make use of it. */
        InitialisePersistentState(StateFileName)  &&
        /* Initialise the connections to the Libera device.  This also needs
         * to be done early, as this is used by other initialisation code. */
        InitialiseHardware()  &&
        /* Get the event receiver up and running.  This spawns a background
         * thread for dispatching trigger events. */
        InitialiseEventReceiver()  &&
        /* Initialise conversion code.  This needs to be done fairly early as
         * it is used globally. */
        InitialiseConvert()  &&
        /* Initialise Libera configuration: switches, attenuators, etc. */
        InitialiseConfigure()  &&
        /* Ensure the trigger interlock mechanism is working. */
        InitialiseTriggers()  &&
        /* Timestamp and clock management. */
        InitialiseTimestamps()  &&

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



/* Shutdown is a little delicate.  We terminate all our threads in an orderly
 * way, but unfortunately we there's not way to synchronise with the EPICS
 * layer: in particular, this means that EPICS threads will continue calling
 * in until _exit() is called.  This means that we don't want to close most
 * resources, so all we really do below is terminate threads. */

static void TerminateLibera()
{
    TerminateEventReceiver();
    TerminateTimestamps();
    TerminateSlowAcquisition();
    TerminatePersistentState();
        
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
"    -v                 Writes version information\n"
"    -p <pid-file>      Writes pid to <pid-file>.\n"
"    -n                 Run non-interactively without an IOC shell\n"
"    -c<key>=<value>    Configure run time parameter.  <key> can be:\n"
"       LT      Length of long turn-by-turn buffer\n"
"       TT      Length of short turn-by-turn buffer\n"
"       TW      Length of turn-by-turn readout window\n"
"       DD      Length of /1024 decimated data buffer\n"
"       HA      Harmonic: number of bunches per revolution\n"
"       DE      Decimation: number of samples per revolution\n"
"       LP      LMTD prescale factor\n"
"    -f <f_mc>          Machine revolution frequency\n"
"    -s <state-file>    Read and record persistent state in <state-file>\n"
"\n"
"Note: This IOC application should normally be run from within runioc.\n",
        IocName);
}



/* Process any options supported by the ioc.  See Usage() for the options
 * supported.  Parameters argc and argv are updated to point past the
 * options. */

static bool ProcessOptions(int &argc, char ** &argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(argc, argv, "+hvp:nc:f:s:"))
        {
            case 'h':   Usage(argv[0]);                 return false;
            case 'v':   StartupMessage();               return false;
            case 'p':   Ok = WritePid(optarg);          break;
            case 'n':   SetNonInteractive();            break;
            case 'c':   Ok = ParseConfigInt(optarg);    break;
            case 'f':   Ok = ParseFloat(
                            optarg, RevolutionFrequency);  break;
            case 's':   StateFileName = optarg;         break;
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



/*****************************************************************************/
/*                                                                           */
/*                        Reboot and Restart Support                         */
/*                                                                           */
/*****************************************************************************/


/* Fairly generic routine to start a new detached process in a clean
 * environment. */

static void DetachProcess(const char *Process, char *const argv[])
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
            execve(Process, argv, envp);
        }
        else
            /* The middle process simply exits without further ceremony.  The
             * idea here is that this orphans the new process, which means
             * that the parent process doesn't have to wait() for it, and so
             * it won't generate a zombie when it exits. */
            _exit(0);
    }
    else
    {
        /* Wait for the middle process to finish. */
        if (waitpid(MiddlePid, NULL, 0) == -1)
            perror("Error waiting for middle process");
    }
}


static void DoReboot()
{
    char * Args[] = { "/sbin/reboot", NULL };
    DetachProcess(Args[0], Args);
}


static void DoRestart()
{
    char * Args[] = { "/etc/init.d/epics", "restart", NULL };
    DetachProcess(Args[0], Args);
}


static void DoCoreDump()
{
    * (char *) 0 = 0;
}


/****************************************************************************/
/*                                                                          */


int main(int argc, char *argv[])
{
    /* A handful of global PVs with no other natural home. */
    Publish_stringin("VERSION", VersionString);
    Publish_stringin("BUILD", BuildDate);
    PUBLISH_ACTION("REBOOT",  DoReboot);
    PUBLISH_ACTION("RESTART", DoRestart);
    PUBLISH_ACTION("CORE", DoCoreDump);
    
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
