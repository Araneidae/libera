/* $Id: lstd.c,v 1.23 2006/12/04 10:42:47 ales Exp $ */

//! \file lstd.c 
//! Implements Libera Syytem Time PLL daemon.

/*
LIBERA PLL DAEMONS - Libera GNU/Linux PLL daemons
Copyright (C) 2004 Instrumentation Technologies

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

TAB = 4 spaces.
*/

#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <sys/select.h>


#include "libera.h"

#include "debug.h"
#include "libera_pll.h"
#include "common.h"

/** Min. number of arguments taken by the application. */
#define MIN_ARGS 1

/** Max. number of arguments taken by the application. */
#define MAX_ARGS 100 // TODO

//--------------------------------------------------------------------------
// Globals.

/** Pointer to the application file name. */
const char *argv0 = 0;

/** Libera device file descriptor. */
int event_fd = -1;

/** Default DAC nominal offset */
unsigned long u_nominal = LSTD_DEFAULT_UNOMINAL;

/** Debug filename */
char *plldebug_fname = "/tmp/lstd_debug.dat";

/** Debug file pointer */
FILE *f_plldebug = NULL;

/** Debug flag */
int plldebug = FALSE;

//--------------------------------------------------------------------------
// Local decls.

/** Signal handler.
 *  Handle SIGINT (Ctrl-C) and other termination signals to allow the
 *  application to terminate gracefully (after cleanup).
 *  @param signo Signal number.
 */
void signal_handler( int signo );

/** Cleanup function.
 *  Remove the process identification (PID) file.
 */
void cleanup();

/** Initialize this instance -- i.e. register signal handler,
 *  atexit handler, create a process identification (PID) file and
 *  daemonize this instance.
 *  Returns 0.
 */
int init();

/** Run the daemon.
 *  Listen on Libera Event device for ??? and ??? triggers and
 *  run the PLL control algorithms.
 *  Returns 0.
 */
int run();

/** Find if a process exists.
 *  Returns 1 (exists) or 0 (does not exist).
 *  @param fname Pointer to pid filename.
 */
int find_instance( const char *fname );

/** Print diagnostic message and exit.
 *  @param function Function name.
 *  @param line Line number.
 *  @param what Error message.
 */
void die( const char *function, int line, const char *what );

/** Print usage information. */
void usage();

/** Print version information. */
void version();

//--------------------------------------------------------------------------

int main( int argc, char *argv[] )
{
    // Make argv0 point to the file name part of the path name.
    argv0 = strrchr( argv[0], '/' );
    if ( argv0 )
    {
        ASSERT( 0 != (argv0+1) );
        argv0 += 1;             // Skip the '/'.
    }
    else argv0 = argv[0];       // Use full pathname instead.

    if ( argc < MIN_ARGS || argc > MAX_ARGS )
    {
        usage();
        exit( EXIT_FAILURE );
    }

    int ch = -1;
    while ( (ch = getopt( argc, argv, "hu:vt:" )) != -1 )
    {
        switch ( ch )
        {
            case 'u':
                u_nominal = atol(optarg);
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
                
            default:
                exit( EXIT_FAILURE );
        }
    }

    init();
    run();

    return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------

int init()
{
    int nochdir = 0, noclose = 0;
    int log_options = LOG_PID;

#if DEBUG       // defined(DEBUG) && DEBUG != 0
    noclose = 1;
    log_options |= LOG_PERROR;  // Print to stderr as well.
#endif          // DEBUG
    
    // Deamonize this process.
    VERIFY( 0 == daemon( nochdir, noclose ) );
    
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
    
    for (int i=0; i<sizeof(sigs)/sizeof(int); ++i)
    {
        if ( 0 != sigaction( sigs[i], &sa, 0 ) ) EXIT( "sigaction" );
    }

    umask(0);

    if ( 0 != find_instance( LSTD_PID_PATHNAME ) )
    {
        _LOG_ERR( "cannot run more than one daemon instance" );
        exit( EXIT_FAILURE );
    }
    
    // Create a pid file before the blocking trigger functions.
    FILE *fp = fopen( LSTD_PID_PATHNAME, "w" );
    if (!fp) EXIT( "fopen" );
    
    fprintf( fp, "%d\n", getpid() );
    if ( 0 != fclose(fp) ) EXIT( "fclose" );
    
    _LOG_DEBUG( "created pid file %s", LSTD_PID_PATHNAME );
    
    
    // Open Libera event device in RDONLY mode. Leave the exclusive
    // access to the event fifo. 
    event_fd = open( LIBERA_EVENT_FIFO_PATHNAME, O_RDONLY );
    if( -1 == event_fd ) EXIT( "open" );
    
    /* lst clock 125MHz */
    if ( 0 != ioctl( event_fd, LIBERA_EVENT_SET_DAC_B, u_nominal ) ) 
        EXIT("ioctl");
    
    // Enable triggers
    if ( 0 > ioctl( event_fd,
                    LIBERA_EVENT_ENABLE_SC_TRIG,
                    TRIGGER_BIT(5) ) ) // M3 SC prescaler = 5
        EXIT("ioctl");

    // Unlocked initially
    unsigned int init_locked = FALSE;    
    if ( 0 > ioctl( event_fd, LIBERA_EVENT_SET_SCPLL, &init_locked) )
        _LOG_CRIT( "failed to set SCPLL" );

    // Debug output
    if (plldebug)
        f_plldebug = fopen(plldebug_fname,"w");
    
    return 0;
}

//--------------------------------------------------------------------------

int find_instance( const char *fname )
{
    FILE *fp = fopen( fname, "r" );
    if ( !fp )
    {
        if ( ENOENT != errno ) EXIT( "fopen" );
        return 0;
    }
    _LOG_WARNING( "found existing pid file %s", fname );

    int rc = 0;
    char *line = 0;
    size_t size = 0;

    if ( -1 != getline( &line, &size, fp ) )
    {
        const pid_t pid = atol(line);
        const int no_signal = 0;
        
        if ( 0 == kill( pid, no_signal ) )
            rc = 1;
        else
            if ( errno != ESRCH ) EXIT( "kill" );
    }
    if ( line ) free( line );

    if ( 0 != fclose(fp) ) EXIT( "fclose" );

    return rc;
}

//--------------------------------------------------------------------------

volatile sig_atomic_t termination_in_progress = 0;

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

//--------------------------------------------------------------------------

void cleanup()
{    
    // Remove pid file
    if ( 0 != unlink( LSTD_PID_PATHNAME ) )
    {
        _LOG_ERR( "failed to unlink %s: %s",
                  LSTD_PID_PATHNAME,
                  strerror( errno ) );
        return;
    }
    
    _LOG_DEBUG( "removed PID file %s", LSTD_PID_PATHNAME );

    // Close Libera event device
    close(event_fd);
    // Close PLL debug file
    if (f_plldebug) fclose(f_plldebug);
}

//--------------------------------------------------------------------------

/** Phase lock helper fcn. */
inline int phase_locked()
{
    unsigned int local_locked = TRUE;

    syslog(LOG_INFO, "Phase locked.\n");

    int ret = ioctl( event_fd, LIBERA_EVENT_SET_SCPLL, &local_locked);
    if ( 0 >  ret)
        _LOG_CRIT( "failed to set SCPLL" );
    
    return ret;
}

/** Phase unlock helper fcn. */
inline int phase_unlocked()
{
    unsigned int local_locked = FALSE;
    
    syslog(LOG_INFO, "Phase unlocked.\n");
    
    int ret = ioctl( event_fd, LIBERA_EVENT_SET_SCPLL, &local_locked);
    if ( 0 >  ret)
        _LOG_CRIT( "failed to set SCPLL" );
    
    return ret;
}

//--------------------------------------------------------------------------

int run()
{
    unsigned int dac = 0;
    unsigned int locked = FALSE;
    unsigned int tIlocked = FALSE;
    unsigned int fIlocked = FALSE;

    unsigned long long sctime_1, sctime_2;
    unsigned long long sc_diff;
    
    long long ext_lmt = 0;
    long long err_lst = 0;
    long long lst_start = 0;
    const double f_ref = 125e6;
    const long long sc_trig_inc = 125e5;
    double f_sc, ext_scf;

    //unlock controller
    double err = 0;
    double var_err = 100.0;
    long long fI = 0;
    double fK = 1.6;


    //I lock regulator
    long long fIl = 0;
    double fKl = 0.5;
    long long tI = 0;
    double tKI = 1;

    double Pphi = 20.0;
    double u_scf;
    int rc;


    // Do not use a _LOG_NOTICE macro!
    // We want the following logged regardless of the current log level.
    syslog( LOG_NOTICE,
            "%s %s configured -- resuming normal operations",
            argv0,
            XSTR(RELEASE_VERSION) );
    
    // Get initial trigger
    do
    {
        rc = ioctl( event_fd,
                    LIBERA_EVENT_GET_SC_TRIGGER_9,
                    &sctime_1 );
    } while ( ((rc == -1) && (errno == EAGAIN)) );
    if (0 > rc) EXIT("ioctl");
    
    /* Main PLL control loop */
    while(1)
    {
        rc = ioctl( event_fd,
                    LIBERA_EVENT_GET_SC_TRIGGER_9,
                    &sctime_2 );
        if (rc < 0 )
        {  // timeout end err check
            if ( !((rc == -1) && (errno == EAGAIN)) )
                _LOG_CRIT( "failed to get SC trigger" );
            if (locked)   // only when changed to unlock 
                phase_unlocked();

            locked = FALSE;
            tIlocked = FALSE;
            fIlocked = FALSE;
            var_err = 100.0;
            fI = 0;
            continue;
        }


        ext_scf = f_ref;        
        sc_diff = sctime_2 - sctime_1;

        f_sc = sc_diff * 10;
        err = ext_scf - f_sc;
        

        /* frequency "unlock" regulator */
        if (!locked)
        {
            var_err = (var_err*5 + err*err)/20;
            fI += err;
        }

        u_scf = u_nominal + fI*fK;


        /* phase regulator */
        if (locked)
        {
            ext_lmt += sc_trig_inc;
            err_lst = ext_lmt - (sctime_2 - lst_start);

            // P regulator
            if ( ( err_lst < -1) || (err_lst > 1) )
                u_scf += err_lst*Pphi;

            // f regulator
            if (fIlocked && (( err_lst >= -1) && (err_lst <= 1)))
            {
                fIl = err_lst;
                u_scf += fIl*fKl;
            }

            // I regulator
            if (tIlocked)
            {
                tI += err_lst;
                u_scf += tI*tKI;
            }

        }

        // Clip, round and set DAC control voltage u_scf
        if (u_scf < 0) u_scf = 0; if (u_scf > 0xFFFF) u_scf=0xFFFF;
        dac=(unsigned int)(u_scf + 0.5);// + (random()/RAND_MAX));
        if ( 0 > ioctl( event_fd, LIBERA_EVENT_SET_DAC_B, dac) )
            _LOG_CRIT( "failed to set DAC B" );

        // Report current phase error to the driver
        if ( 0 > ioctl( event_fd, LIBERA_EVENT_SET_SCPHI, &err_lst) )
            _LOG_CRIT( "failed to set SCPHI" );

        // Debug output
        if (plldebug)
        {
            fprintf(f_plldebug,"%u %lld %lld %15.8f %15.8f %15.8f %15.8f \n",
                    dac, sc_diff, err_lst, ext_scf,
                    f_sc, err, var_err);
            fflush(f_plldebug);
        }


        if ( (var_err < 10) && !locked )
        {
            locked = TRUE;
            tIlocked = TRUE;
            fIlocked = TRUE;

            phase_locked();

            ext_lmt = 0;
            err_lst = 0;
            lst_start = sctime_2;
            tI = 0;
            fIl = 0;
        };

        // lock the t integrator
        if ( (( err_lst > -3000) && (err_lst < 3000)) && !tIlocked && locked )
        {
            tIlocked = TRUE;

            _LOG_DEBUG( "Time integrator unlocked.\n" );

            tI = 0;
        }

        // unlock the t integrator
        if ( (( err_lst < -4000) || (err_lst > 4000)) && tIlocked )
        {
            tIlocked = FALSE;

            _LOG_DEBUG( "Time integrator unlocked.\n" );
        }

        // lock the f integrator
        if ( (( err > -500) && (err < 500)) && !fIlocked && locked )
        {
            fIlocked = TRUE;

            _LOG_DEBUG( "Frequency integrator locked.\n" );

            fIl = 0;
        }

        // unlock the f integrator
        if ( (( err < -1000) || (err > 1000)) && fIlocked )
        {
            fIlocked = FALSE;

            _LOG_DEBUG( "Frequency integrator unlocked.\n" );
        }

        // unlock the phase
        if ( (( err_lst < -ERR_LST_UNLOCK) ||
              (err_lst > ERR_LST_UNLOCK)) && locked )
        {
            locked = FALSE;
            tIlocked = FALSE;
            fIlocked = FALSE;

            phase_unlocked();

            var_err = 100.0;
            fI = 0;
        }


        if ( (( dac < 5000) || (dac > 60000)) && !locked )
            fI = 0;

        sctime_1 = sctime_2;
    } 

    return 0;
}

//--------------------------------------------------------------------------

void die( const char *function, int line, const char *what )
{
    syslog( LOG_CRIT,
        "system error in function `%s': line %d: `%s' -- %s",
        function,
        line,
        what,
        ( errno ? strerror(errno) : "(n/a)" ) );
    
    exit( EXIT_FAILURE );
}

//--------------------------------------------------------------------------

void usage()
{
    const char *format =
    "Usage: %s [OPTION]...\n"
    "\n"
    "-o u_nominal    DAC nominal offset (default = 0x%x)\n"
    "-t file         Test mode. Write debug signals to file.\n"
    "-h              Print this message and exit.\n"
    "-v              Print version information and exit.\n"
    "\n";

    fprintf( stderr, format, argv0, u_nominal );
}

//--------------------------------------------------------------------------

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
