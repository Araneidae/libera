/* This file is part of the Libera EPICS Driver,
 * 
 * Copyright (C) 2008 Michael Abbott, Diamond Light Source Ltd.
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

/* Clock PLL Daemon: application level startup code.  Spawns separate threads
 * for machine and system clocks and defines common framework code. */

#define _GNU_SOURCE
#include <stdbool.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <syslog.h>
#include <pthread.h>

#include "libera_pll.h"
#include "test_error.h"

#include "machineClock.h"
#include "systemClock.h"
#include "clockPll.h"



/* Controls whether the this runs as a daemon.  Set to false for debug. */
static bool daemon_mode = true;

/* File handle to /dev/libera.event: all access to clocks is through ioctl
 * commands on this handle. */
int event_fd = -1;



/*****************************************************************************/
/*                                                                           */
/*                             Error Logging                                 */
/*                                                                           */
/*****************************************************************************/

/* Routine for printing an error message complete with associated file name
 * and line number. */
/* This is stolen from liberaApp/src/hardware.cpp and belongs in a shared
 * library. */

void print_error(const char * Message, const char * FileName, int LineNumber)
{
    /* Large enough not to really worry about overflow.  If we do generate a
     * silly message that's too big, then that's just too bad. */
    const int MESSAGE_LENGTH = 512;
    int Error = errno;
    char ErrorMessage[MESSAGE_LENGTH];
    
    int Count = snprintf(ErrorMessage, MESSAGE_LENGTH,
        "%s (%s, %d)", Message, FileName, LineNumber);
    if (errno != 0)
    {
        /* This is very annoying: strerror() is not not necessarily thread
         * safe ... but not for any compelling reason, see:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00101.html
         * and the rather unhelpful reply:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00108.html
         *
         * On the other hand, the recommended routine strerror_r() is
         * inconsistently defined -- depending on the precise library and its
         * configuration, it returns either an int or a char*.  Oh dear.
         *
         * Ah well.  We go with the GNU definition, so here is a buffer to
         * maybe use for the message. */
        char StrError[256];
        snprintf(ErrorMessage + Count, MESSAGE_LENGTH - Count,
            ": (%d) %s", Error, strerror_r(Error, StrError, sizeof(StrError)));
    }
    if (daemon_mode)
        syslog(LOG_ERR, "%s", ErrorMessage);
    else
        fprintf(stderr, "%s\n", ErrorMessage);
}


void log_message(int Priority, const char * Format, ...)
{
    va_list args;
    va_start(args, Format);
    char Message[128];
    vsnprintf(Message, sizeof(Message), Format, args);
    
    if (daemon_mode)
        syslog(Priority, "%s", Message);
    else
        printf("%d: %s\n", Priority, Message);
}



/*****************************************************************************/
/*                                                                           */
/*                         Status and Command Pipes                          */
/*                                                                           */
/*****************************************************************************/


/* Status reports are written here. */
static int status_pipe = -1;

/* Mutex for serialising writes to the status pipe. */
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;
/* The status pipe can overflow: this flag records whether this has happened,
 * in which case we'll write a special command when we can.  We start with
 * this set to true to generate a reset command on startup. */
static bool status_pipe_overflow = true;



/* Ensures that the required fifo resources are already available. */

bool InitialiseCommandLoop()
{
    return
        /* Create the command and status FIFOs. */
        TEST_(mkfifo, CLOCK_PLL_COMMAND_FIFO, 0666)  &&
        TEST_(mkfifo, CLOCK_PLL_STATUS_FIFO,  0666)  &&
        /* Open the status FIFO right away.  We do this now so that the
         * machine and system clock threads can write freely.
         *    The fifo is opened read-only first before opening write only as
         * a special hack !!!??? why??? */
        TEST_(open, CLOCK_PLL_STATUS_FIFO, O_RDONLY | O_NONBLOCK)  &&
        TEST_IO(status_pipe,
            open, CLOCK_PLL_STATUS_FIFO, O_WRONLY | O_NONBLOCK);
}



/* Dispatch incoming commands to the appropriate handler. */

void DispatchCommand(char *Command)
{
    char * Newline = strchr(Command, '\n');
    if (Newline == NULL)
        log_message(LOG_ERR, "Malformed command \"%s\"", Command);
    else
    {
        *Newline = '\0';
        switch (Command[0])
        {
            case 'm':   MachineClockCommand(Command + 1);       break;
            case 's':   SystemClockCommand(Command + 1);        break;
            case 'n':   SetNcoFrequency(atoi(Command + 1));     break;
            default:
                log_message(LOG_ERR, "Unknown command \"%s\"", Command);
        }
    }
}


/* Processing incoming commands on the command pipe, one command per line.
 * Each time the pipe is closed it is reopened. */

bool RunCommandLoop()
{
    while (true)
    {
        FILE * CommandPipe = fopen(CLOCK_PLL_COMMAND_FIFO, "r");
        if (TEST_OK(CommandPipe != NULL))
        {
            char Command[80];
            while (fgets(Command, sizeof(Command), CommandPipe) != NULL)
                DispatchCommand(Command);
            fclose(CommandPipe);
        }
        else
            /* Oops.  This really shouldn't have happened. */
            return false;
    }
}



/* Sends a status message to the listening client. */

void WriteStatus(const char *Format, ...)
{
    va_list args;
    va_start(args, Format);
    char Message[128];
    vsnprintf(Message, sizeof(Message), Format, args);
    
    /* It turns out that although POSIX specifies that write is thread safe,
     * in fact this is not to be relied on, so we write under a lock.  The
     * story is here: http://lwn.net/Articles/180387/ and the referenced
     * patch is not in the current Kernel. */
    TEST_0(pthread_mutex_lock, &status_mutex);
    if (status_pipe_overflow)
        /* The x command is interpreted as loss of connection by the client. */
        write(status_pipe, "x\n", 2);
    int written = write(status_pipe, Message, strlen(Message));
    status_pipe_overflow = (size_t) written != strlen(Message);
    TEST_0(pthread_mutex_unlock, &status_mutex);
}



/*****************************************************************************/
/*                                                                           */
/*                           Clock PLL Startup                               */
/*                                                                           */
/*****************************************************************************/

/* There are very few options supported by this process: everything else is
 * controlled through the command pipe. */

static bool ProcessOptions(int argc, char *argv[], MC_PARAMETERS *Params)
{
    int ch;
    while(ch = getopt(argc, argv, "+p:d:r:n"), ch != -1)
    {
        switch (ch)
        {
            case 'p':   Params->Prescale   = atol(optarg);  break;
            case 'd':   Params->Decimation = atol(optarg);  break;
            case 'r':   Params->Harmonic   = atol(optarg);  break;
            case 'n':   daemon_mode = false;                break;
            default:
                return false;
        }
    }
    argc -= optind;
    return
        TEST_OK(argc == 0)  &&
        TEST_OK(Params->Prescale != 0)  &&
        TEST_OK(Params->Decimation != 0) &&
        TEST_OK(Params->Harmonic != 0);
}



/* The signal handler shuts the process down without any ceremony.  We rely
 * on normal OS handling to clean up the residue. */

void ExitHandler(int signo) __attribute__((noreturn));
void ExitHandler(int signo)
{
    /* Similarly destroy the command and status pipes. */
    unlink(CLOCK_PLL_COMMAND_FIFO);
    unlink(CLOCK_PLL_STATUS_FIFO);
    /* Make sure we don't leave the PID file behind -- do this last of all. */
    unlink(CLOCK_PLL_PID_FILE);
    /* Die NOW! */
    _exit(0);
}


/* Intercept the usual signals for killing the process and place a PID file
 * so that we can be killed in an orderly way while running as a daemon. */

bool InitialiseExitHandler()
{
    struct sigaction AtExitHandler;
    AtExitHandler.sa_handler = ExitHandler;
    AtExitHandler.sa_flags = 0;
    
    int PidFile;
    char Pid[32];
    return
        /* Block all signals during AtExit() signal processing. */
        TEST_(sigfillset, &AtExitHandler.sa_mask)  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_(sigaction, SIGHUP,  &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGINT,  &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGQUIT, &AtExitHandler, NULL)  &&
        TEST_(sigaction, SIGTERM, &AtExitHandler, NULL)  &&
        
        /* Try to create a new PID file.  If it already exists then we'll
         * fail without any further fuss. */
        TEST_IO(PidFile,
            open, CLOCK_PLL_PID_FILE, O_WRONLY | O_CREAT | O_EXCL, 0644)  &&

        /* At this point we push ourself into the background if required.
         * This needs to be done after testing for the PID file but before we
         * actually compute the PID, as the daemon() call will change our
         * process id! */
        (! daemon_mode  ||  TEST_(daemon, false, false))  &&

        sprintf(Pid, "%d", getpid())  &&
        TEST_(write, PidFile, Pid, strlen(Pid))  &&
        TEST_(close, PidFile);
}



int main(int argc, char *argv[])
{
    MC_PARAMETERS McParams;
    bool Ok =
        /* Process command line arguments. */
        ProcessOptions(argc, argv, &McParams)  &&
        /* Sort out shutdown handling. */
        InitialiseExitHandler();
    if (!Ok)
        return 1;

    /* Finally spawn the PLL threads before running the command loop. */
    Ok =
        TEST_IO(event_fd, open, "/dev/libera.event", O_RDONLY)  &&
        /* Need to initialise the status loop resources before spawning the
         * threads, as they'll be using our resources. */
        InitialiseCommandLoop()  &&
        InitialiseMachineClock(&McParams)  &&
        InitialiseSystemClock()  &&

        /* Finally run the command status loop.  This shouldn't return. */
        RunCommandLoop();
    
    /* If we get up here then forcibly clean up any dangling resources.  Note
     * that this never returns. */
    ExitHandler(0);
}
