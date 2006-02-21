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
#include "slowAcquisition.h"

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


/* Configuration settable parameters. */

/* Maximum length of long turn by turn buffer. */
static int LongTurnByTurnLength = 200000;
static int ShortTurnByTurnLength = 2048;
static int TurnByTurnWindowLength = 16384;
/* Length of 1024 decimated buffer. */
static int DecimatedShortLength = 190;


/* Prints interactive startup message as recommended by GPL. */

void StartupMessage()
{
    printf(
"Libera EPICS Driver, Version " LIBERA_VERSION ",\n"
"Copyright (C) 2005-2006 Michael Abbott, Diamond Light Source\n"
"This program comes with ABSOLUTELY NO WARRANTY.  This is free software,\n"
"and you are welcome to redistribute it under certain conditions.\n"
"For details see the GPL or the attached file COPYING.\n");
}



/* We take any of the four traditional shutdown signals (HUP, INT, QUIT or
 * TERM) as a request to terminate the IOC.  We let the IOC terminate in an
 * orderly manner, so the task is to interrupt whatever the main ioc loop is
 * up to.
 *     Note that this code is in a signal handler, so we must take care to
 * only call async-safe functions.  This rather restricts what we're allowed
 * to do! */

void AtExit(int signal)
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

bool InitialiseAtExit()
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


static void SetNonInteractive()
{
    close(0);
    RunIocShell = false;
    sem_init(&ShutdownSemaphore, 0, 0);
}



/* Prints usage message in response to -h option. */

void Usage(const char *IocName)
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
"       TL      Length of long turn-by-turn buffer\n"
"       TT      Length of short turn-by-turn buffer\n"
"       TW      Length of turn-by-turn readout window\n"
"       DD      Length of /1024 decimated data buffer\n",
        IocName);
}


/* The Libera driver is started by starting all of its constituent components
 * in turn.  Here is the natural place for these to be defined. */

bool InitialiseLibera()
{
    return
        /* Set up exit hander. */
        InitialiseAtExit()  &&
        
        /* Initialise the connections to the Libera device. */
        InitialiseHardware()  &&
        /* Initialise conversion code.  This needs to be done fairly early as
         * it is used globally. */
        InitialiseConvert()  &&
        /* Get the event receiver up and running.  This spawns a background
         * thread for dispatching trigger events. */
        InitialiseEventReceiver()  &&

        /* Now we can initialise the mode specific components. */

        /* First turn processing is designed for transfer path operation. */
        InitialiseFirstTurn()  &&
        /* Turn by turn is designed for long waveform capture at revolution
         * clock frequencies. */
        InitialiseTurnByTurn(LongTurnByTurnLength, ShortTurnByTurnLength)  &&
        /* Booster operation is designed for viewing the entire booster ramp
         * at reduced resolution. */
        InitialiseBooster(DecimatedShortLength)  &&
        /* Slow acquisition returns highly filtered positions at 10Hz. */
        InitialiseSlowAcquisition();
}


/* Shutting down is really just a matter of closing down the event receiver
 * (because it runs a separate thread: this needs to be done in an orderly
 * way, otherwise we'll crash) and closing our connections to the Libera
 * driver (to be tidy). */

void TerminateLibera()
{
    /* On orderly shutdown remove the pid file if we created it. */
    if (PidFileName != NULL)
        unlink(PidFileName);
    TerminateEventReceiver();
    TerminateSlowAcquisition();
    TerminateHardware();
}



/* Write the PID of this process to the given file. */

bool WritePid(const char * FileName)
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

bool ParseConfigInt(char *optarg)
{
    static const struct
    {
        const char * Name;
        int * Target;
        int Low, High;
    } Lookup[] = {
        { "TL", & LongTurnByTurnLength,   1, 500000 },  // Up to 16M
        { "TT", & ShortTurnByTurnLength,  1, 8192 },
        { "TW", & TurnByTurnWindowLength, 1, 65536 },   // Up to 8M
        { "DD", & DecimatedShortLength,   1, 1000 },
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
    if (*eq == '\0'  ||  *end != '\0')
    {
        printf("Configuration value not a number: \"%s=%s\"\n", optarg, eq);
        return false;
    }
    
    /* Figure out who it belongs to! */
    for (size_t i = 0; i < sizeof(Lookup) / sizeof(Lookup[0]); i ++)
    {
        if (strcmp(optarg, Lookup[i].Name) == 0)
        {
            if (Lookup[i].Low <= Value  &&  Value <= Lookup[i].High)
            {
                *Lookup[i].Target = Value;
                return true;
            }
            else
            {
                printf("Unreasonable value: \"%s=%s\"\n", optarg, eq);
                return false;
            }
        }
    }

    /* Nope, never heard of it. */
    printf("Unknown configuration value \"%s\"\n", optarg);
    return false;
}


/* Process any options supported by the ioc.  At present we support:
 *
 *      -h              Print out usage
 *      -p<filename>    Write ioc PID to <filename>
 *
 * argc and argv are updated to point past the options. */

bool ProcessOptions(int &argc, char ** &argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(argc, argv, "+p:nc:hv"))
        {
            case 'p':   Ok = WritePid(optarg);          break;
            case 'n':   SetNonInteractive();            break;
            case 'c':   Ok = ParseConfigInt(optarg);    break;
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
    /* Consume any option arguments and start the driver. */
    bool Ok =
        ProcessOptions(argc, argv)  &&
        InitialiseLibera();

    /* Consume any remaining script arguments by running them through the IOC
     * shell. */
    for (; Ok  &&  argc > 0; argc--)
        Ok = iocsh(*argv++) == 0;

    /* Run the entire IOC with a live IOC shell. */
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
