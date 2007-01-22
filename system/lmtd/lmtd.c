/* $Id: lmtd.c,v 1.34 2006/07/13 12:00:46 ales Exp $ */

//! \file lmtd.c 
//! Implements Libera Machine Time PLL Daemon.

/*
LIBERA PLL DAEMONS - Libera GNU/Linux PLL daemons
Copyright (C) 2004-2006 Instrumentation Technologies
Copyright (C) 2006-2007 Michael Abbott, Diamond Light Source Ltd.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
or visit http://www.gnu.org
*/

#define _GNU_SOURCE
#include <stdbool.h>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <sys/select.h>

#include <pthread.h>

#include "libera.h"

#include "debug.h"
#include "libera_pll.h"




//--------------------------------------------------------------------------
// Globals.

/** Pointer to the application file name: this is used to label log entries. */
const char *argv0 = 0;

/** Libera device file descriptor, used to interface to the machine clock
 *  control . */
int event_fd = -1;

/** Default MC precaler */
unsigned long mc_presc = LMTD_DEFAULT_MCPRESC;

/** Default decimation */
unsigned long ddc_decimation = LMTD_DEFAULT_DEC;

/* Nominal number of ticks expected between successive clock count samples.
 * Always equal to mc_presc*ddc_decimation. */
unsigned int system_prescale = LMTD_DEFAULT_MCPRESC * LMTD_DEFAULT_DEC;

/** Default DAC nominal offset */
unsigned long u_nominal = LMTD_DEFAULT_UNOMINAL;

/** Compensated offset-tune flag (NCO shift) */
int nco_shift = FALSE;

/** Debug filename */
char *plldebug_fname = "/tmp/lmtd_debug.dat";

/** Debug file pointer */
FILE *f_plldebug = NULL;

/** Debug flag */
int plldebug = FALSE;
bool daemon_mode = true;


/* Default RF VCXO detuning offset.  This may be dynamically changed during
 * operation. */
long frequency_offset = 0;

long phase_offset = 0;
long nco_offset = 0;

bool synchronised = false;

int status_pipe = -1;



// #ifdef NO_DAEMON
// /* Nasty hackery: when not running as a daemon, send any error reports
//  * directly to the terminal!  Otherwise they just go into system log. */
// #define syslog(level, format, ...) \
//     ( { \
//         printf("syslog %d\n", level); \
//         printf(format, ##__VA_ARGS__); \
//     } )
// #endif



/* Print diagnostic message and exit.  Called from EXIT() macro.
 *  @param function Function name.
 *  @param line Line number.
 *  @param what Error message. */

void die(const char *function, int line, const char *what)
{
    syslog(LOG_CRIT,
        "system error in function `%s': line %d: `%s' -- %s",
        function, line, what, errno ? strerror(errno) : "(n/a)");
    
    exit(EXIT_FAILURE);
}



/*****************************************************************************/
/*                                                                           */
/*                    Command and Control and Reporting                      */
/*                                                                           */
/*****************************************************************************/



//--------------------------------------------------------------------------

void ParseCommandInt(const char *Command, long *Target)
{
    char * End;
    int NewValue = strtol(Command + 1, &End, 10);
    if (Command + 1 < End  &&  *End == '\0')
        /* Good, read a new value. */
        *Target = NewValue;
    else
        _LOG_CRIT("Invalid number in %c command: \"%s\"\n",
            Command[0], Command + 1);
}


void* CommandThread(void*Context)
{
    mkfifo(LMTD_COMMAND_FIFO, 0666);
    while (true)
    {
        int CommandPipe = open("/tmp/lmtd.command", O_RDONLY);
        if (CommandPipe == -1)  EXIT("Unable to open command pipe");
        char Command[80];
        int Read = read(CommandPipe, Command, sizeof(Command) - 1);
        if (Read > 0)
        {
            Command[Read] = '\0';
            printf("Read \"%s\"\n", Command);
            switch (Command[0])
            {
                case 'o':
                    ParseCommandInt(Command, &frequency_offset);
                    break;
                case 'p':
                    ParseCommandInt(Command, &phase_offset);
                    break;
                case 'n':
                    ParseCommandInt(Command, &nco_offset);
                    break;
                case 's':
                    synchronised = true;
                    break;

                default:
                    _LOG_ERR("Unknown command \"%s\"\n", Command);
            }
        }
        else
            _LOG_ERR("Reading command (%d)", errno);
        close(CommandPipe);
    }
    return NULL;
}



typedef enum
{
    LMTD_NO_CLOCK,          // Clock lost
    LMTD_FREQUENCY_SEEK,    // Seeking requested frequency
    LMTD_PHASE_SEEK,        // Wide band phase lock
    LMTD_PHASE_LOCKED       // Narrow band phase lock
} LMTD_STATE;

#define PHASE_LOCKED(State) \
    ((State) == LMTD_PHASE_SEEK  ||  (State) == LMTD_PHASE_SEEK)

/* Log messages on transition between each of the states above. */
const char * LmtdStateLogMessage[4][4] = {
    { NULL,          "Clock lost",  "Clock lost",       "Clock lost" },
    { "Clock found", NULL,          "Phase lock lost",  "Error 3->1" },
    { "Error 0->2",  "Phase locked", NULL,              "Phase slip" },
    { "Error 0->3",  "Error 1->3",   "Full phase lock", NULL }
};

LMTD_STATE LastKnownState = LMTD_NO_CLOCK;
bool PipeOverflow = false;


/* This reports the LMTD state.  The full state is written to the status
 * pipe, and significant changes in state are reported to the log and the
 * device driver.
 *     The following state is reported:
 *
 *      Basic loop state:
 *              no clock, seeking frequency, phase locked, deep phase lock
 *      Loop error:
 *              frequency error when frequency seeking, phase error in phase
 *              seek and locked modes.
 *      DAC setting
 */

void ReportLmtdState(LMTD_STATE LmtdState, int LoopError, int DAC)
{
    /* Report state changes to log file and driver. */
    const char * LogMessage = LmtdStateLogMessage[LmtdState][LastKnownState];
    if (LogMessage != NULL)
        /* Report new state to system log. */
        syslog(LOG_INFO, LogMessage);

    /* Ensure the device driver has an accurate picture of the phase lock
     * state. */
    if (PHASE_LOCKED(LmtdState) != PHASE_LOCKED(LastKnownState))
    {
        unsigned int Locked = PHASE_LOCKED(LmtdState);
        if (ioctl(event_fd, LIBERA_EVENT_SET_MCPLL, &Locked) == -1)
            _LOG_CRIT("Failed to set MCPLL");
    }
        
    LastKnownState = LmtdState;

    
    /* Prepare status message to send to monitor. */
    char StatusMessage[80];
    sprintf(StatusMessage, "%s%c %d %d\n",
        PipeOverflow ? "O\n" : "",
        "XfpL"[LmtdState], LoopError, DAC);

    /* Write message to pipe. */
    PipeOverflow =
        write(status_pipe, StatusMessage, strlen(StatusMessage)) 
            != (int) strlen(StatusMessage);
}



/* All commands are received on a separate thread. */

void InitialiseCommandThread(void)
{
    /* Create thread to receive commands. */
    pthread_t ThreadId;
    pthread_create(&ThreadId, NULL, CommandThread, NULL);

    /* Create the status FIFO ready to receive our status reports. */
    mkfifo(LMTD_STATUS_FIFO, 0666);
    open(LMTD_STATUS_FIFO, O_RDONLY | O_NONBLOCK);
    status_pipe = open(LMTD_STATUS_FIFO, O_WRONLY | O_NONBLOCK);
}


//--------------------------------------------------------------------------

/*****************************************************************************/
/*                                                                           */
/*                          Frequency Locked Loop                            */
/*                                                                           */
/*****************************************************************************/

//--------------------------------------------------------------------------

/* Returns the current absolute machine time and (if Counts is not NULL) the
 * (estimated) number of tick intervals since the last machine time.  When
 * Counts is non-NULL then *MachineTime must record the previous machine time.
 *
 * This mechanism is currently required because the device driver can block
 * for a very long time, causing counts to become lost. */

bool GetMachineTime(unsigned long long * MachineTime)
{
    unsigned long long LastMachineTime = *MachineTime;
    
    /* Read the machine time.  This ioctl will block until the a machine time
     * can be read (100ms), or a timeout occurs, in which case the ioctl will
     * fail and errno is set to EAGAIN.
     *    For strange (probably historical) reasons we also need to read the
     * SC trigger, even though we immediately throw it away.  If we don't,
     * the log fills up with SC overflow messages! */
    ioctl(event_fd, LIBERA_EVENT_GET_SC_TRIGGER_10, MachineTime);
    bool mc_ok =
        ioctl(event_fd, LIBERA_EVENT_GET_MC_TRIGGER_10, MachineTime) == 0;

    /* Normally either the ioctl succeeded, or it failed with a timeout:
     * almost certainly because the machine clock trigger isn't connected.
     * We only want to log something if neither of these cases holds. */
    if (!mc_ok  &&  errno != EAGAIN)
        /* Hmm.  Log something. */
        _LOG_CRIT("Unable to get MC trigger: %s", strerror(errno));

    if (!mc_ok)
        printf("Clock lost\n");

    if (mc_ok)
    {
        unsigned long long McTimeDelta = *MachineTime - LastMachineTime;
        int Count =
            (int) ((McTimeDelta + system_prescale/2) / system_prescale);

        static struct timespec Before;
        struct timespec Now;
        clock_gettime(CLOCK_REALTIME, &Now);
        int ns_delta = Now.tv_nsec - Before.tv_nsec;
        int s_delta = Now.tv_sec - Before.tv_sec;
        if (ns_delta < 0)
        {
            ns_delta += 1000000000;
            s_delta -= 1;
        }

         if (Count != 1 || s_delta != 0 ||
             80000000 > ns_delta || ns_delta > 120000000)
//        if (Count != 1)
        {
            printf(
                "Delta = %llu, delta = %d.%09d, count=%d, residue = %lld\n",
                McTimeDelta, 
                s_delta, ns_delta, Count,
                McTimeDelta - Count * system_prescale);
        }
        Before = Now;
    }
    
    return mc_ok;
}



/* PLL loop state.  The control loop runs in two main modes, unlocked and
 * locked.  When the loop is unlocked it searches for the right frequency,
 * but makes no attempt to track phase and the control loop is very simple.
 * When the loop is locked it tracks the clock phase since the locking point
 * and uses a PID control loop to ensure that the phase is tracked with
 * minimum deviation. */


int ClipDAC(int dac)
{
    if (dac < 0)
        dac = 0;
    else if (dac >= 0xFFFF)
        dac = 0xFFFF;
    return dac;
}


void SetMachineClockDAC(int dac)
{
    if (ioctl( event_fd, LIBERA_EVENT_SET_DAC_A, dac) != 0)
        _LOG_CRIT( "failed to set DAC A" );
}

void ReportFrequency(unsigned long long mcdiff)
{
    unsigned long fmc_set = (unsigned long) (100 * mcdiff);
    if (ioctl(event_fd, LIBERA_EVENT_SET_FLMC, &fmc_set) != 0)
        _LOG_CRIT( "failed to set f_lmc" );
}


void ReportPhase(long long mcphi)
{
    if (ioctl(event_fd, LIBERA_EVENT_SET_MCPHI, &mcphi) != 0)
        _LOG_CRIT( "failed to set ERR_LMT" );
}



/* Control constants for PLL. */

/* Frequency locking. */
#define FF_FK   18      // Frequency scale
#define FF_IIR  0.5     // Lock detect IIR factor

/* Phase searching. */
#define FP_KP   20      // Proportionality constant
#define FP_IIR  0.25

/* Phase locking. */
#define LP_KP   0.5
#define LP_KI   0.05
#define LP_IIR  0.05


/* This routine tunes the MC clock until the correct frequency is found.  As
 * soon as the frequency has settled true is returned together with the
 * machine time and the corresponding DAC setting.  If the clock is lost
 * during this process false is returned instead. */

bool run_find_frequency(unsigned long long *mctime, int *target_dac)
{
    /* Smoothed squared error for lock detection. */
    double var_err = 1e2;
    /* Computed DAC output. */
    int dac = *target_dac;

    unsigned long long mctime_1 = *mctime;
    unsigned long long mctime_2 = mctime_1;
    while (GetMachineTime(&mctime_2))
    {
        /* The frequency error is determined by the difference. */
        unsigned long long mc_diff = mctime_2 - mctime_1;
        /* The frequency error is determined by the prescale (which
         * determines the nominal number of ticks) together with the
         * frequency offset. */
        int error = system_prescale + frequency_offset - mc_diff;
        dac = ClipDAC(dac + FF_FK * error);

        ReportLmtdState(LMTD_FREQUENCY_SEEK, error, dac);
        SetMachineClockDAC(dac);
        mctime_1 = mctime_2;

        /* Also run a simple IIR to detect when the error stabilises for long
         * enough.  If the error stabilises small enough, we can now lock. */
        var_err = FF_IIR * (double) error * error + (1 - FF_IIR) * var_err;
        if (var_err < 1.5)
        {
            *mctime = mctime_2;
            *target_dac = dac;
            return true;
        }
    }
    
    /* If we fall through to here then machine time was lost. */
    ReportLmtdState(LMTD_NO_CLOCK, 0, dac);
    return false;
}


/* This is the first part of the phase locked loop for the machine clock.
 * This part of the loop is designed to seek the target phase as quickly as
 * possible without worrying too much about long term phase stability.
 *    On successful capture of the desired phase true is returned together
 * with the updated nominal machine time and the current DAC setting.
 *    If phase is lost or if the clock is lost false is returned. */

bool run_find_phase(unsigned long long *mctime, int *dac)
{
    /* This is the expected nominal clock count which we will maintain.  The
     * clock count is set when the lock is acquired and accumulated on each
     * clock tick. */
    unsigned long long nominal_clock_count = *mctime;
    
    /* Integrated error. */
    int tI = 0;

    /* Smoothed squared error for lock detection. */
    double var_err = 1e2;

    /* All DAC computations will be offsets from the nominal DAC set on
     * entry. */
    int nominal_dac = *dac;
    
//    unsigned long long mctime_1 = *mctime;
    unsigned long long new_mctime = *mctime;
    while (GetMachineTime(&new_mctime))
    {
        /* Accumulate the clock count and compute the corresponding phase
         * error.  Here we take the programmed frequency and phase offsets
         * into account. */
        nominal_clock_count += system_prescale + frequency_offset;
        long long mcphi = nominal_clock_count - new_mctime;
        int error = (int) (mcphi - phase_offset);

        tI += error;
        *dac = ClipDAC(nominal_dac + FP_KP * error + tI);

        ReportPhase(mcphi);
        ReportLmtdState(LMTD_PHASE_SEEK, error, *dac);
        SetMachineClockDAC(*dac);
//         ReportFrequency(new_mctime - mctime_1);
//         mctime_1 = new_mctime;

        
        // unlock the phase
        if (abs(error) > ERR_LMT_UNLOCK)
        {
            *mctime = new_mctime;
            return false;
        }

        
        var_err = FP_IIR * error * error + (1 - FP_IIR) * var_err;
//printf("find_phase: %d (%g) => %d\n", error, var_err, dac);
        if (var_err < 2)
        {
            *mctime = nominal_clock_count;
            return true;
        }
    }
    
    /* If we fall through to here the machine time was lost. */
    ReportLmtdState(LMTD_NO_CLOCK, 0, *dac);
    return false;
}



/* This runs a narrow bandwidth filter to keep the LMT phase locked as
 * closely as possible.  Long term filtering is performed and a very long
 * time-constant is used on the integrator.
 *    If false is returned, the clock has been lost.  If true is returned,
 * the phase error is too large for the narrow bandwidth lock, and the wider
 * bandwidth find_phase process needs to be run instead. */

bool run_lock_phase(unsigned long long *mctime, int *dac)
{
    /* Accumulated target phase. */
    unsigned long long nominal_clock_count = *mctime;

    /* Running average of errors.  We just use a simple IIR for this. */
    double smoothed_error = 0;
    /* Integrator. */
    int integrated_error = 0;

    int nominal_dac = *dac;
    unsigned long long new_mctime = *mctime;
    while (GetMachineTime(&new_mctime))
    {
        nominal_clock_count += system_prescale + frequency_offset;
        long long mcphi = nominal_clock_count - new_mctime;
        int this_error = (int) (mcphi - phase_offset);

        integrated_error += this_error;
        smoothed_error = LP_IIR * this_error + (1 - LP_IIR) * smoothed_error;
        *dac = ClipDAC(nominal_dac + (int) round(
            LP_KP * smoothed_error + LP_KI * integrated_error));

        ReportPhase(mcphi);
        ReportLmtdState(LMTD_PHASE_LOCKED, this_error, *dac);
        SetMachineClockDAC(*dac);
//        ReportFrequency(new_mctime - mctime_1);

        /* Check for drift of phase error.  If it gets more than about +-1
         * then this filter is too tight and we need to find the base phase
         * again. */
        if (abs(this_error) > 3)
        {
            /* Phase error too big.  Drop back to the faster filter. */
            *mctime = nominal_clock_count;
            return true;
        }
    }
    
    /* If we fall through to here the machine time was lost. */
    ReportLmtdState(LMTD_NO_CLOCK, 0, *dac);
    return false;
}


void run()
{
    /* This loop implements locking the clock in four increasing stages:
     *  1. Capture machine time.  When the machine clock is disconnected all
     *     we can do is report this.
     *  2. Tune to the correct frequency.  During this process we cannot hope
     *     to lock the phase.
     *  3. Slew to the correct phase.  A fast but not particularly accurate
     *     algorithm is used to ensure we get to the right phase rapidly.
     *  4. Lock to the requested phase.  Once the phase is locked we can run
     *     very slow and tightly tuned filters.
     *
     * At each stage the process can bail out to the stage above: this
     * occurs when the phase becomes unlocked or when the machine clock is
     * lost. */
    int dac = u_nominal;
    while (true)
    {
        unsigned long long mctime;
        while (!GetMachineTime(&mctime))
            /* At this point we are completely untied: there is no external
             * machine clock available, so there is nothing we can do except
             * report this. */
            ReportLmtdState(LMTD_NO_CLOCK, 0, dac);

        /* Alternately acquire frequency and then lock the phase.  While the
         * machine is locked we maintain the machine time, and the dac is also
         * passed between states to preserve stability. */
        while (run_find_frequency(&mctime, &dac))       // Seek frequency
        {
            while (run_find_phase(&mctime, &dac)  &&    // Lock into phase
                   run_lock_phase(&mctime, &dac))       // Maintain phase lock
                ;
        }
    }
}



/*****************************************************************************/
/*                                                                           */
/*                          Daemon Initialisation                            */
/*                                                                           */
/*****************************************************************************/


/** Find if there is another instance of the lmtd already running, returns
 ** true iff found. */

bool find_instance( const char *fname )
{
    FILE *pid_file = fopen( fname, "r" );
    if (pid_file == NULL)
        /* No pid file, assume no other process running. */
        return false;
    else
    {
        _LOG_WARNING( "found existing pid file %s", fname );
        bool Found = false;
        char *line = NULL;
        size_t size = 0;
        if (getline(&line, &size, pid_file) != -1)
        {
            /* Probe for the existence of the process whos pid is recorded in
             * the file. */
            pid_t pid = atol(line);
            Found = pid > 0  &&  kill(pid, 0) == 0;
        }
        if (line != NULL)
            free(line);

        fclose(pid_file);

        return Found;
    }
}

//--------------------------------------------------------------------------

/** Cleanup function.
 *  Remove the process identification (PID) file. */

void cleanup()
{    
    // Remove pid file
    if ( 0 != unlink( LMTD_PID_PATHNAME ) )
    {
        _LOG_ERR( "failed to unlink %s: %s",
              LMTD_PID_PATHNAME,
              strerror( errno ) );
        return;
    }
    
    _LOG_DEBUG( "removed PID file %s", LMTD_PID_PATHNAME );

    // Close Libera event device
    close(event_fd);
    // Close PLL debug file
    if (f_plldebug) fclose(f_plldebug);

    /* Remove the lmtd fifos. */
    unlink(LMTD_COMMAND_FIFO);
    unlink(LMTD_STATUS_FIFO);
}

//--------------------------------------------------------------------------

volatile sig_atomic_t termination_in_progress = 0;

/** Signal handler.
 *  Handle SIGINT (Ctrl-C) and other termination signals to allow the
 *  application to terminate gracefully (after cleanup).
 *  @param signo Signal number. */

void signal_handler( int signo )
{
    // Since this handler is established for more than one kind of signal,
    // it might still get invoked recursively by delivery of some other kind
    // of signal. Use a static variable to keep track of that.
    if ( termination_in_progress ) raise( signo );
    termination_in_progress = 1;

    // Do not use a _LOG_NOTICE macro!
    // We want the following logged regardless of the current log level.
    syslog( LOG_NOTICE, "caught signal %d, shutting down", signo );

    // Now do the cleanup.
    cleanup();

    // Next, restore the signal's default handling and reraise the signal to
    // terminate the process.
    _LOG_INFO( "re-raising signal %d", signo );

    signal( signo, SIG_DFL );
    raise( signo );
}



/** Initialize this instance -- i.e. register signal handler,
 *  atexit handler, create a process identification (PID) file and
 *  daemonize this instance.
 *  Returns 0. */

int init()
{
    /* First check that we're not running already: if we are, simply refuse
     * to start. */
    if (find_instance(LMTD_PID_PATHNAME))
    {
        fprintf(stderr, "Lmtd is already running\n");
        _LOG_ERR( "cannot run more than one daemon instance" );
        exit( EXIT_FAILURE );
    }
    
    int noclose = 0;
    int log_options = LOG_PID;
#if DEBUG   // defined(DEBUG) && DEBUG != 0
    fprintf(stderr, "Starting test lmtd\n");
    noclose = 1;
    log_options |= LOG_PERROR;  // Print to stderr as well.
#endif      // DEBUG

    if (daemon_mode)
        // Deamonize this process.
        VERIFY(0 == daemon(0, noclose));
    
    // Note: closelog() is optional and therefore not used.
    openlog( argv0, log_options, 0  );
    
    // Install cleanup handler.
    VERIFY( 0 == atexit( cleanup ) );
    
    // Setup signal handler.
    struct sigaction sa;
    sigemptyset( &sa.sa_mask );
    sa.sa_handler = signal_handler;
    
    // Handle Ctrl-C and regular termination requests.
    const int sigs[] = { SIGINT, SIGHUP, SIGTERM, SIGQUIT };
    
    for (unsigned int i=0; i<sizeof(sigs)/sizeof(int); ++i)
    {
        if ( 0 != sigaction( sigs[i], &sa, 0 ) )
            EXIT( "sigaction" );
    }

    umask(0);

    // Create a pid file before the blocking trigger functions.
    FILE *fp = fopen( LMTD_PID_PATHNAME, "w" );
    if (!fp) EXIT( "fopen" );
    
    fprintf( fp, "%d\n", getpid() );
    if ( 0 != fclose(fp) ) EXIT( "fclose" );
    
    _LOG_DEBUG( "created pid file %s", LMTD_PID_PATHNAME );
    
    
    // Open Libera event device in RDONLY mode. Leave the exclusive
    // access to the event fifo. 
    event_fd = open( LIBERA_EVENT_FIFO_PATHNAME, O_RDONLY );
    if( -1 == event_fd ) EXIT( "open" );
    
    // Enable triggers
    if ( 0 > ioctl( event_fd,
            LIBERA_EVENT_ENABLE_SC_TRIG,
            TRIGGER_BIT(6) ) ) // M3 prescaler = 6
    EXIT("ioctl");
    if ( 0 > ioctl( event_fd,
            LIBERA_EVENT_ENABLE_MC_TRIG,
            TRIGGER_BIT(6) ) ) // M3 prescaler = 6
    EXIT("ioctl");
    
    // Unlocked initially
    unsigned int init_locked = FALSE;    
    if ( 0 > ioctl( event_fd, LIBERA_EVENT_SET_MCPLL, &init_locked) )
        _LOG_CRIT( "failed to set MCPLL" );

    // Debug output
    if (plldebug)
        f_plldebug = fopen(plldebug_fname,"w");

    InitialiseCommandThread();
    
    return 0;
}



/*****************************************************************************/
/*                                                                           */
/*                          Command Line Parsing                             */
/*                                                                           */
/*****************************************************************************/


/** Print usage information. */

void usage()
{
    const char *format =
    "Usage: %s [OPTION]...\n"
    "\n"
    "-d decimation   Decimation factor (default = %lu)\n"
    "-o offset-tune  RF-VCXO detuning offset (*40Hz), integer (default = %lu)\n"   
    "-c              Compensate tune; Shifts NCO accoording to RF-VCXO.\n"
    "-u u_nominal    DAC nominal offset (default = 0x%x)\n"
    "-p prescaler    MC prescaler (default = %lu)\n"
    "-t file         Test mode. Write debug signals to file.\n"
    "-n              Non-daemon: do not run as a daemon, debug mode.\n"
    "-h              Print this message and exit.\n"
    "-v              Print version information and exit.\n"
    "\n";

    fprintf( stderr, format, argv0,
         ddc_decimation,
         frequency_offset,
         u_nominal,
         mc_presc);
}



/** Print version information. */

void version()
{
    const char *format =
    "%s %s (%s %s)\n"
    "\n"
    "Copyright 2004, 2005 Instrumentation Technologies.\n"
    "This is free software; see the source for copying conditions. "
    "There is NO warranty; not even for MERCHANTABILITY or FITNESS "
    "FOR A PARTICULAR PURPOSE.\n\n";

    printf( format, argv0, XSTR(RELEASE_VERSION), __DATE__, __TIME__ );
}



/* Main entry point. */

int main(int argc, char *argv[])
{
    /* Make argv0 point to the file name part of the path name. */
    argv0 = strrchr( argv[0], '/' );
    if ( argv0 )
        argv0 += 1;             // Skip the '/'.
    else
        argv0 = argv[0];        // Use full pathname instead.

    int ch;
    while (
        ch = getopt(argc, argv, "d:f:hco:p:u:vt:n"),
        ch != -1)
    {
        switch ( ch )
        {
        case 'p':
            mc_presc = atol(optarg);
            break;

        case 'd':
            ddc_decimation = atol(optarg);
            break;

        case 'u':
            u_nominal = atol(optarg);
            break;
            
        case 'o':
            frequency_offset = atol(optarg);
            break;
            
        case 'c':
            nco_shift = TRUE;
            break;

        case 'h':
            usage();
            exit( EXIT_SUCCESS );
            
        case 'v':
            version();
            exit( EXIT_SUCCESS );
            
        case 't':
            plldebug = TRUE;
            plldebug_fname = optarg;
            break;

        case 'n':
            daemon_mode = false;
            break;

        default:
            exit( EXIT_FAILURE );
        }
    }
    argc -= optind;
    argv += optind;
    if (argc > 0)
    {
        printf("Unexpected extra arguments\n");
        exit(EXIT_FAILURE);
    }

    /* Compute system_prescale. */
    system_prescale = mc_presc * ddc_decimation;

    init();
    run();

    return EXIT_SUCCESS;
}
