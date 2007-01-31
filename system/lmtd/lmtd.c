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
#include "common.h"




//--------------------------------------------------------------------------
// Globals.

/* Pointer to the application file name: this is used to label log entries. */
const char *argv0 = 0;

/* Libera device file descriptor, used to interface to the machine clock
 * control . */
int event_fd = -1;

/* Default MC precaler: number of machine clocks between MC tick events. */
unsigned long mc_presc = LMTD_DEFAULT_MCPRESC;

/* Default decimation: number of samples per revolution */
unsigned long ddc_decimation = LMTD_DEFAULT_DEC;

/* Default harmonic number: number of bunches per revolution  */
unsigned long harmonic = LMTD_DEFAULT_HARMONIC;

/* Nominal number of ticks expected between successive clock count samples.
 * Always equal to mc_presc*ddc_decimation. */
unsigned int system_prescale = LMTD_DEFAULT_MCPRESC * LMTD_DEFAULT_DEC;

/* Default DAC nominal offset */
unsigned long u_nominal = LMTD_DEFAULT_UNOMINAL;

/* Maximum allowable phase error before we unlock phase. */
long MaximumPhaseError = ERR_LMT_UNLOCK;

/* Debug filename */
char *plldebug_fname = "/tmp/lmtd_debug.dat";
/* Debug file pointer */
FILE *f_plldebug = NULL;
/* Debug flag */
int plldebug = FALSE;

/* Whether to detach as a daemon: set to false for debug. */
bool daemon_mode = true;


/* Default RF VCXO detuning offset.  This may be dynamically changed during
 * operation. */
long frequency_offset = 0;

/* Phase offset.  Only really meaningful after clock synchronisation. */
long phase_offset = 0;

/* We allow the NCO offset to differ from the frequency offset for special
 * applications, but normally these should coincide. */
long nco_offset = 0;

/* Synchronisation state. */
bool synchronised = false;

/* Pipe used to receive status messages. */
int status_pipe = -1;





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





#define PHASE_LOCKED(State) \
    ((State) == LMTD_PHASE_SEEK  ||  (State) == LMTD_PHASE_SEEK)

/* Log messages on transition between each of the states above.  We only log
 * certain transitions.  In particular, transitions between the two phase
 * locked states aren't logged. */
const char * LmtdStateLogMessage
    [LMTD_LOCK_STATE_COUNT][LMTD_LOCK_STATE_COUNT] =
{
    { NULL,          "Clock lost",  "Clock lost",       "Clock lost" },
    { "Clock found", NULL,          "Phase lock lost",  "Error 3->1" },
//     { "Error 0->2",  "Phase locked", NULL,              NULL },
//     { "Error 0->3",  "Error 1->3",   NULL,              NULL }
    { "Error 0->2",  "Phase locked", NULL,              "Phase slip" },
    { "Error 0->3",  "Error 1->3",   "Full phase lock", NULL }
};


LMTD_LOCK_STATE LastKnownState = LMTD_NO_CLOCK;
bool PipeOverflow = false;



/* This reports the LMTD state.  The full state is written to the status
 * pipe, and significant changes in state are reported to the log and the
 * device driver. */

void ReportLmtdState(
    LMTD_LOCK_STATE LmtdState, int FrequencyError, int PhaseError, int DAC)
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
    sprintf(StatusMessage, "%d %d %d %d\n",
        LmtdState, FrequencyError, PhaseError, DAC);

    /* Write message to pipe. */
    PipeOverflow =
        write(status_pipe, StatusMessage, strlen(StatusMessage)) 
            != (int) strlen(StatusMessage);
}



/*****************************************************************************/
/*                                                                           */
/*                          Interface to Machine                             */
/*                                                                           */
/*****************************************************************************/


/* Returns the current absolute machine time and (if Counts is not NULL) the
 * (estimated) number of tick intervals since the last machine time.  When
 * Counts is non-NULL then *MachineTime must record the previous machine time.
 *
 * This mechanism is currently required because the device driver can block
 * for a very long time, causing counts to become lost. */

bool GetMachineTime(unsigned long long * MachineTime)
{
    /* This ioctl returns the number of 125MHz system clock ticks at the
     * current MC tick report.  We don't use this (though it could be used to
     * infer the machine frequency), but it needs to be read otherwise the
     * device driver will complain.  This data stream is probably a historical
     * relic. */
    unsigned long long SystemTime;
    ioctl(event_fd, LIBERA_EVENT_GET_SC_TRIGGER_10, &SystemTime);
    
    /* Read the machine time.  This ioctl will block until the a machine time
     * can be read (100ms), or a timeout occurs, in which case the ioctl will
     * fail and errno is set to EAGAIN. */
    bool mc_ok =
        ioctl(event_fd, LIBERA_EVENT_GET_MC_TRIGGER_10, MachineTime) == 0;

    /* Normally either the ioctl succeeded, or it failed with a timeout:
     * almost certainly because the machine clock trigger isn't connected.
     * We only want to log something if neither of these cases holds. */
    if (!mc_ok  &&  errno != EAGAIN)
        _LOG_CRIT("Unable to get MC trigger: %s", strerror(errno));

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



/* The phase advance per sample for the intermediate frequency generator is
 * controlled by an ioctl.  The phase advance is in units of 2^32*f_if/f_s,
 * where f_if is the desired intermediate frequency and f_s is the sample clock
 * frequency.
 *     If we write P=prescale, D=decimation, H=bunches per turn and
 * F=frequency offset then the sample clock satisfies:
 *
 *          f_s = (D/H + F/HP) f_rf
 *
 * We normally want to set f_if = f_rf (modulo f_s), in which case the desired
 * intermediate frequency scaling factor (to ensure that the resampled RF
 * frequency is equal to the IF) is
 *                                                HP
 *          N = 2^32 frac(f_rf/f_s) = 2^32 frac ------
 *                                              PD + F
 */

void SetNcoFrequency()
{
    /* We need to calculate the fractional part of f_rf/f_s to get the correct
     * value for N.  As the frequency offset F is always quite small (and is
     * guaranteed to be less than frac(H/D)), we can accurately calculate the
     * integer part of HP/(PD+F) as the integer part of H/D. */
    unsigned long nco = (unsigned long) (
        (double)(1ULL << 32) * (
            ((double) harmonic * mc_presc) /
                ((double) system_prescale + nco_offset) -
            harmonic / ddc_decimation));
    if (ioctl(event_fd, LIBERA_EVENT_SET_NCO, &nco) != 0)
        _LOG_CRIT( "failed to set NCO" );
}



/*****************************************************************************/
/*                                                                           */
/*                          Frequency Locked Loop                            */
/*                                                                           */
/*****************************************************************************/


/* This routine tunes the MC clock until the correct frequency is found.  As
 * soon as the frequency has settled true is returned together with the
 * machine time and the corresponding DAC setting.  If the clock is lost
 * during this process false is returned instead. */

#define FF_FK   20      // Frequency scale

bool run_find_frequency(unsigned long long *mctime, int *target_dac)
{
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
        int frequency_error = system_prescale + frequency_offset - mc_diff;
        dac = ClipDAC(dac + FF_FK * frequency_error);

        ReportLmtdState(LMTD_FREQUENCY_SEEK, frequency_error, 0, dac);
        SetMachineClockDAC(dac);
        mctime_1 = mctime_2;

        /* Return once the target frequency is reached. */
        if (abs(frequency_error) <= 1)
        {
            *mctime = mctime_2;
            *target_dac = dac;
            return true;
        }
    }
    
    /* If we fall through to here then machine time was lost. */
    ReportLmtdState(LMTD_NO_CLOCK, 0, 0, dac);
    return false;
}


/* This is the first part of the phase locked loop for the machine clock.
 * This part of the loop is designed to seek the target phase as quickly as
 * possible without worrying too much about long term phase stability.
 *    On successful capture of the desired phase true is returned together
 * with the updated nominal machine time and the current DAC setting.
 *    If phase is lost or if the clock is lost false is returned. */

/* These P&I constants are chosen to match the open loop gain of the VCXO
 * DAC, which is approximately 0.03 sample clocks per ~10Hz MC tick per unit
 * of DAC setting.
 *    These values are chosen for a reasonably rapid DAC response with
 * reduced risk of instability. */
#define FP_KP   20      // Proportionality constant
#define FP_KI   9       // Integration constant

/* This "lock in" time constant is used to determine how long we wait before
 * handing on to the narrow band locked loop. */
#define FP_IIR  0.15    // Lock in time constant

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

    unsigned long long new_mctime = *mctime;
    unsigned long long last_mctime = new_mctime;
    while (GetMachineTime(&new_mctime))
    {
        /* Accumulate the clock count and compute the corresponding phase
         * error.  Here we take the programmed frequency and phase offsets
         * into account. */
        nominal_clock_count += system_prescale + frequency_offset;
        long long mcphi = nominal_clock_count - new_mctime;
        int phase_error = (int) (mcphi - phase_offset);
        tI += phase_error;

        *dac = ClipDAC(nominal_dac + FP_KP * phase_error + FP_KI * tI);

        /* If the DAC hits the limits we have a problem.  If we let the
         * integrator continue to run then we end up overcompensating, and and
         * then oscillating for ages afterwards. If, on the other hand, we
         * simply reset the integrater then we can oscillate forever if we
         * bounce off the limits.  Thus here we simply don't integrate this
         * term -- seems to work. */
        if (*dac == 0  ||  *dac == 0xFFFF)
            tI -= phase_error;

        /* Compute the frequency error, just as in run_find_frequency, so
         * that we can report how the frequency changes as we slew. */
        int frequency = new_mctime - last_mctime;
        last_mctime = new_mctime;
        int frequency_error = system_prescale + frequency_offset - frequency;
        ReportFrequency(frequency);

        ReportPhase(mcphi);
        ReportLmtdState(LMTD_PHASE_SEEK, frequency_error, phase_error, *dac);
        SetMachineClockDAC(*dac);
        
        /* If the phase error grows too large give up trying to hold the
         * locked phase and hand control back to the frequency seeking code.
         * */
        if (abs(phase_error) > MaximumPhaseError)
        {
            *mctime = new_mctime;
            return false;
        }

        /* Finally check for stable phase lock: once the phase lock is
         * sufficiently stable, we can hand off to the narrow lock filter. */
        var_err = FP_IIR * phase_error * phase_error + (1 - FP_IIR) * var_err;
        if (var_err < 2)
        {
            *mctime = nominal_clock_count;
            return true;
        }
    }
    
    /* If we fall through to here the machine time was lost. */
    ReportLmtdState(LMTD_NO_CLOCK, 0, 0, *dac);
    return false;
}



/* A couple of experimental filters. */
//#define FILTER_SMOOTH_PI
#define FILTER_SECOND_ORDER



/* This runs a narrow bandwidth filter to keep the LMT phase locked as
 * closely as possible.  Long term filtering is performed and a very long
 * time-constant is used on the integrator.
 *    If false is returned, the clock has been lost.  If true is returned,
 * the phase error is too large for the narrow bandwidth lock, and the wider
 * bandwidth find_phase process needs to be run instead. */

#ifdef FILTER_SMOOTH_PI
#define LP_KP   0.5
#define LP_KI   0.05
#endif
#ifdef FILTER_SECOND_ORDER
/* These filter coefficients define a second order IIR filter which is used to
 * managed the phase error.  The goal is to keep the phase error low (to
 * within +-1 or 2 sample clocks) with neither excessive excursions in
 * frequency or long term oscillations -- it turns out that designing such a
 * filter is quite tricky.  The coefficients below work for a system with an
 * open loop gain of approximately 0.03. */
#if 0
#define B_0     0.97
#define B_1     -1.1
#define B_2     0.17
#else
#define B_0     0.3
#define B_1     0.14
#define B_2     -0.41
#endif
#define A_1     -1.8
#define A_2     0.8
#endif

#define LP_IIR  0.05

bool run_lock_phase(unsigned long long *mctime, int *dac)
{
    /* Accumulated target phase. */
    unsigned long long nominal_clock_count = *mctime;

#ifdef FILTER_SMOOTH_PI
    /* Running average of errors.  We just use a simple IIR for this. */
    double smoothed_error = 0;
    /* Integrator. */
    int integrated_error = 0;
#endif
#ifdef FILTER_SECOND_ORDER
    /* Second order IIR.  We have to keep a history of the last two terms and
     * the last two corrections. */
    int   last_error[2] = { 0, 0 };
    float last_out  [2] = { 0.0, 0.0 };
#endif

    int nominal_dac = *dac;
    unsigned long long new_mctime = *mctime;
    while (GetMachineTime(&new_mctime))
    {
        nominal_clock_count += system_prescale + frequency_offset;
        long long mcphi = nominal_clock_count - new_mctime;
        int this_error = (int) (mcphi - phase_offset);

#ifdef FILTER_SMOOTH_PI
        integrated_error += this_error;
        smoothed_error = LP_IIR * this_error + (1 - LP_IIR) * smoothed_error;
        *dac = ClipDAC(nominal_dac + (int) round(
            LP_KP * smoothed_error + LP_KI * integrated_error));
#endif
#ifdef FILTER_SECOND_ORDER
        /* Compute this stage of the filter. */
        float this_output =
            B_0 * this_error + B_1 * last_error[0] + B_2 * last_error[1] -
            A_1 * last_out[0] - A_2 * last_out[1];
        /* Advance the historical records. */
        last_out[1]   = last_out[0];    last_out[0]   = this_output;
        last_error[1] = last_error[0];  last_error[0] = this_error;
        /* Compute the required correction to output for this step. */
        *dac = ClipDAC(nominal_dac + (int) round(this_output));
#endif

        ReportPhase(mcphi);
        ReportLmtdState(LMTD_PHASE_LOCKED, 0, this_error, *dac);
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
    ReportLmtdState(LMTD_NO_CLOCK, 0, 0, *dac);
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
            ReportLmtdState(LMTD_NO_CLOCK, 0, 0, dac);

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
/*                         LMTD Command Processing                           */
/*                                                                           */
/*****************************************************************************/


void DispatchCommand(char *Command)
{
    char * Newline = strchr(Command, '\n');
    if (Newline == NULL)
        _LOG_ERR("Malformed command \"%s\"", Command);
    else
    {
        *Newline = '\0';
printf("Read \"%s\"\n", Command);
        switch (*Command++)
        {
            case 'o':
                frequency_offset = atoi(Command);
                break;
            case 'p':
                phase_offset = atoi(Command);
                break;
            case 'n':
                nco_offset = atoi(Command);
                SetNcoFrequency();
                break;
            case 's':
                synchronised = true;
                break;
            case 'w':
                MaximumPhaseError = atoi(Command);
                break;

            default:
                _LOG_ERR("Unknown command \"%s\"", Command-1);
        }
    }
}



void* CommandThread(void*Context)
{
    mkfifo(LMTD_COMMAND_FIFO, 0666);
    while (true)
    {
        FILE * CommandPipe = fopen(LMTD_COMMAND_FIFO, "r");
        if (CommandPipe == NULL)
        {
            _LOG_ERR("Error opening command pipe");
            /* Wait 10 seconds before trying again.  Means we don't give up,
             * but also means we can flood the error log... */
            sleep(10);
        }
        else
        {
            char Command[80];
            while (fgets(Command, sizeof(Command), CommandPipe) != NULL)
                DispatchCommand(Command);
            fclose(CommandPipe);
        }
    }
    return NULL;
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




/*****************************************************************************/
/*                                                                           */
/*                          Daemon Initialisation                            */
/*                                                                           */
/*****************************************************************************/



/* Find if there is another instance of the lmtd already running, returns
 * true iff found. */

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



/* Cleanup function.
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



volatile sig_atomic_t termination_in_progress = 0;

/* Signal handler.
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



/* Initialize this instance -- i.e. register signal handler, atexit handler,
 * create a process identification (PID) file and daemonize this instance.
 * Returns 0. */

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

    /* Ensure the intermediate frequency NCO is set to the appropriate
     * frequency. */
    SetNcoFrequency();
    
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


/* Print usage information. */

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



/* Print version information. */

void version()
{
    const char *format =
    "%s %s (%s %s)\n"
    "\n"
    "Copyright 2004-2006 Instrumentation Technologies.\n"
    "Copyright 2006-2007 Michael Abbott, Diamond Light Source Ltd.\n"
    "This is free software; see the source for copying conditions.\n"
    "There is NO warranty; not even for MERCHANTABILITY or FITNESS\n"
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

    bool nco_shift = false;
    int ch;
    while (
        ch = getopt(argc, argv, "d:f:hco:p:r:u:vt:n"),
        ch != -1)
    {
        switch (ch)
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
                nco_shift = true;
                break;

            case 'r':
                harmonic = atol(optarg);
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
    if (nco_shift)
        /* If -c is not selected, the default offset is zero. */
        nco_offset = frequency_offset;

    init();
    run();

    return EXIT_SUCCESS;
}
