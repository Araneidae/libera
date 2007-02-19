// $Id: dscd.c,v 1.7 2006/11/30 15:31:55 ales Exp $

//! \file dscd.c
//! Implements DSC Daemon.

/*
CSPI DSC Daemon
Copyright (C) 2003-2006 Instrumentation Technologies, Slovenia

This program is licenced software; you can use it under the terms of the
Instrumentation Technologies License. You should have received a copy of the
Licence along with this program; if not, write to the Instrumentation
Technologies, Velika pot 22, 5250 Solkan, Slovenia

This program source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY to the extend permitted by applicable law.

All rights reserved.
Any copying, distribution and/or disclosure prohibited.

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

#include <linux/limits.h>	// PATH_MAX

#include "cspi.h"
#include "debug.h"

#include "dscd_impl.h"
#include "cordic_dsc.h"

/* Min. number of arguments taken by the application. */
#define MIN_ARGS 1

/* Max. number of arguments taken by the application. */
#define MAX_ARGS 20

/* Helper macro to stringify the expanded argument. */
#define XSTR(s) STR(s)

/* Stringification macro. */
#define STR(s) #s

/* Helper macro. Print diagnostic system message and exit. */
#define EXIT(what) die( __FUNCTION__, __LINE__, what )

/* Helper macro. Return larger of a and b. */
#define MAX(a,b) ((a)>(b) ? a : b)

/* Helper macro. Return lesser of a and b. */
#define MIN(a,b) ((a)<(b) ? a : b)

/* Defines symbolic names for read and write end of a pipe. */
enum {
	RD = 0,
	WR = 1
};

//--------------------------------------------------------------------------
// Globals.

/* Pointer to the application file name. */
const char *argv0 = 0;

/* Default decimation factor. */
size_t _DEC = 220;
/* Default RF frequency [Hz].*/
double _f_TBT = 533818.3761;
/* Default number of TBT samples per switch position. */
size_t _N_TBT = 40;
/* Default harmonic number. */
size_t _HARMONIC = 936;
/* Sum of attenuators @0dBm [dB]. */
size_t _ATTNSUM_0dBm = 44;
/* ADC-rate buffer peak level @0dBm [ADC count]. */
size_t _ADCPEAK_0dBm = 1228;
/* TBT marker delay for compensation of DDC propagation delay (in ADC samples)*/
size_t _TBT_M_DELAY = 440;
/* analog to digital switch propagation delay  (in ADC samples)*/
size_t _A2D_DELAY = 40;
/* number of full sitching periods for phase compensation calculations */
size_t _PH_AVG = 10;
/* _tune_offset */
size_t _tune_offset = 0;
/* compensated tune offset (Y/N) */
size_t _comp_tune = 0;
/* Machine Clock prescaler */
size_t _MC_presc = 53382;


// Time consumption calculus variables
clock_t ticks1, ticks2;
char *var_time_01, *var_time_02;
char *ctime(const time_t *timer);
time_t timer_start;
int kkk;
// Time consumption calculus variables


//--------------------------------------------------------------------------
// Local decls.

/* Signal handler.
 * Handle SIGINT (Ctrl-C) and other termination signals to allow the
 * application to terminate gracefully (after cleanup).
 */
void sig_handler( int signo );

/* Cleanup function.
 * Remove the process identification (PID) file.
 */
void cleanup();

/* Initialize this instance -- i.e. register signal handler,
 * atexit handler, create a process identification (PID) file and
 * daemonize this instance.
 * Returns 0.
 */
int init();

/* Run the daemon in a simple iterative fashion.
 * Listen to client requests on a fifo (named pipe).
 * Returns 0.
 */
int run();

/* Called on request fifo timeout to call the next compensation
 * method in a row.
 * Returns value returned by the compensation method.
 */
int on_timeout();

/* Handle a request from the client.
 * On success, returns 0. On error, returns -1.
 */
int on_message( message *p );

/* Read up to ntotal bytes from a file descriptor.
 * On a subsequent call(s), attempts to read the missing bytes.
 * On success, returns the number of bytes read.
 * On error, returns -1 (errno may be set to any of the errors
 * specified for the routine read).
 * The nleft argument is set to the number of bytes left to read.
 * @param fd File descriptor.
 * @param buf Pointer to destination buffer.
 * @param ntotal The number of bytes to read.
 * @param nleft The number of left to be read.
 */
int readsome( int fd, void *buf, const size_t ntotal, size_t *nleft );

/* Find if a process exists.
 * Returns 1 (exists) or 0 (does not exist).
 * @param fname Pointer to pid filename.
 */
int find_instance( const char *fname );

/* Print diagnostic message and exit. */
void die( const char *function, int line, const char *what );

/* Print usage information. */
void usage();

/* Print version information. */
void version();

//--------------------------------------------------------------------------

int main( int argc, char *argv[] )
{
	// Make argv0 point to the file name part of the path name.
	argv0 = strrchr( argv[0], '/' );
	if ( argv0 ) {

		ASSERT( 0 != (argv0+1) );
		argv0 += 1;		// Skip the '/'.
	}
	else argv0 = argv[0];	// Use full pathname instead.

	if ( argc < MIN_ARGS || argc > MAX_ARGS ) {

		usage();
		exit( EXIT_FAILURE );
	}

	int ch = -1;
	while ( (ch = getopt( argc, argv, "a:cd:f:g:hm:n:o:p:r:s:t:v" )) != -1 )
	{
		switch ( ch ) {
			
			case 'a':
				_ADCPEAK_0dBm = atoi( optarg );
				break;

			case 'c':
				_comp_tune = atoi( optarg );
				break;

			case 'd':
				_DEC = atoi( optarg );
				break;

			case 'f':
				_f_TBT = atof( optarg );
				break;
				
			case 'g':
				_PH_AVG = atoi( optarg );
				break;				

			case 'h':
				usage();
				exit( EXIT_SUCCESS );

			case 'm':
				_TBT_M_DELAY = atoi( optarg );
				break;
				
			case 'n':
				_N_TBT = atoi( optarg );
				break;

			case 'o':
				_tune_offset = atoi( optarg );
				break;

			case 'p':
				_MC_presc = atoi( optarg );
				break;

			case 'r':
				_HARMONIC = atoi( optarg );
				break;
				
			case 's':
				_ATTNSUM_0dBm = atoi( optarg );
				break;
				
			case 't':
				_A2D_DELAY = atoi( optarg );
				break;
								
			case 'v':
				version();
				exit( EXIT_SUCCESS );

			default:
				exit( EXIT_FAILURE );
				
		}
	}

	if ( 0 == init() ) run();
	return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------

int init()
{
	int nochdir = 0, noclose = 0;
	int log_options = LOG_PID;

#if DEBUG	// defined(DEBUG) && DEBUG != 0
	noclose = 1;
	log_options |= LOG_PERROR;	// Print to stderr as well.
#endif		// DEBUG

	// Deamonize this process.
	VERIFY( 0 == daemon( nochdir, noclose ) );

	// Note: closelog() is optional and therefore not used.
	openlog( argv0, log_options, 0  );

	// Install cleanup handler.
	VERIFY( 0 == atexit( cleanup ) );

	// Setup signal handler.
	struct sigaction sa;
	sigemptyset( &sa.sa_mask );
	sa.sa_handler = sig_handler;

	// Handle Ctrl-C and regular termination requests.
	const int sigs[] = { SIGINT, SIGHUP, SIGTERM, SIGQUIT };
	for (int i=0; i<sizeof(sigs)/sizeof(int); ++i) {

		if ( 0 != sigaction( sigs[i], &sa, 0 ) ) EXIT( "sigaction" );
	}

	umask(0);
	// OK if fifo already exists.
	if ( mkfifo( DSCD_FIFO_PATHNAME, 0666 ) && EEXIST != errno) {

		EXIT( DSCD_FIFO_PATHNAME );
	}

	if ( 0 != find_instance( DSCD_PID_PATHNAME ) ) {

		_LOG_ERR( "cannot run more than one daemon instance" );
		exit( EXIT_FAILURE );
	}

	// Finally, create a pid file.
	FILE *fp = fopen( DSCD_PID_PATHNAME, "w" );
	if (!fp) EXIT( "fopen" );

	fprintf( fp, "%d\n", getpid() );
	if ( 0 != fclose(fp) ) EXIT( "fclose" );
	_LOG_DEBUG( "created pid file %s", DSCD_PID_PATHNAME );

	return init_compensation();
}

//--------------------------------------------------------------------------

int find_instance( const char *fname )
{
	FILE *fp = fopen( fname, "r" );
	if ( !fp ) {

		if ( ENOENT != errno ) EXIT( "fopen" );
		return 0;
	}
	_LOG_WARNING( "found existing pid file %s", fname );

	int rc = 0;
	char *line = 0;
	size_t size = 0;

	if ( -1 != getline( &line, &size, fp ) ) {

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

void sig_handler( int signo )
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
	exit_compensation();

	// Remove pid file
	if ( 0 != unlink( DSCD_PID_PATHNAME ) ) {

		_LOG_ERR( "failed to unlink %s: %s",
		          DSCD_PID_PATHNAME,
		          strerror( errno ) );
		return;
	}
	_LOG_DEBUG( "removed PID file %s", DSCD_PID_PATHNAME );
}

//--------------------------------------------------------------------------

int run()
{
	// Prevent open to block if the other end not open.
	int fd = open( DSCD_FIFO_PATHNAME, O_RDONLY|O_NONBLOCK );
	// Write end is never used. If we do not open the fifo for writing,
	// then each time a client terminates, the fifo becomes empty and the
	// server's read returns 0 to indicate end-of-file. This is a trick to
	// prevent having to reopen the fifo on each client request.
	if ( -1 == fd ||
	     -1 == open( DSCD_FIFO_PATHNAME, O_WRONLY ) ) EXIT( "open" );

	// Do not use a _LOG_NOTICE macro!
	// We want the following logged regardless of the current log level.
	syslog( LOG_NOTICE,
	        "%s %s configured -- resuming normal operations",
	        argv0,
	        XSTR(RELEASE_VERSION) );

	fd_set rfds;
	struct timeval timeout = { DSCD_ITER_PERIOD, 0 };

	message msg;
	size_t nleft = 0;

	while (1) {

		FD_ZERO( &rfds );
		FD_SET( fd, &rfds );

		// Wait for new request or timeout.
		int rc = select( fd + 1, &rfds, 0, 0, &timeout );
		if ( -1 == rc ) {

			if ( EINTR != errno ) EXIT( "select" );
			continue;
		}
		if ( 0 == rc ) {

			timeout.tv_sec = DSCD_ITER_PERIOD;
			timeout.tv_usec = 0;
			
			#if DEBUG
			timer_start = time(NULL);
			ticks1=clock();
			#endif // DEBUG

			on_timeout();
			
			#if DEBUG
			ticks2=clock();
			syslog(LOG_INFO, "----> COMPENSATIONS  - TIME IN SECONDS: %5.10f ", ((double)(ticks2-ticks1))/CLOCKS_PER_SEC );
			#endif // DEBUG
		}
		else {

			ASSERT( FD_ISSET( fd, &rfds ) );

			int nread = readsome( fd, &msg, sizeof(message), &nleft );
			if ( -1 == nread && EINTR != errno ) EXIT("readsome");

			if ( 0 == nleft && on_message( &msg ) ) {

				_LOG_ERR( "cannot handle request" );
			}
		}
	}
	return 0;
}

//--------------------------------------------------------------------------

int on_timeout()
{
	static size_t idx = 0;
	typedef int (*TASK_FNC)();

	const TASK_FNC const tasklist[] = {
		compensate_gain,
		compensate_amplitude,
		compensate_phase,
		compensate_crosstalk,
	};

	const size_t count = sizeof(tasklist)/sizeof(TASK_FNC);
	return (tasklist[idx++ % count])();
}

//--------------------------------------------------------------------------

int readsome( int fd, void *buf, const size_t ntotal, size_t *nleft )
{
	ASSERT( buf );
	ASSERT( nleft );
	ASSERT( *nleft <= ntotal );

	// Reset nleft if we just completed reading ntotal bytes.
	if ( 0 == *nleft ) *nleft = ntotal;
	const ssize_t nread = read( fd, buf + (ntotal-*nleft), *nleft );

	if ( nread > 0 ) *nleft -= nread;
	return nread;
}

//--------------------------------------------------------------------------

static inline int is_valid_message( const message *p )
{
	const int no_signal = 0;

	return ( p->magic == DSCD_MAGIC ) &&
	       ( p->type > DSCD_FIRST && p->type < DSCD_LAST ) &&
	       ( 0 == kill( p->pid, no_signal ) || EPERM == errno);
}

//--------------------------------------------------------------------------

int on_message( message *p )
{
	_LOG_INFO("on_message %d %d %d", p->type, p->val, p->status); // debug only !!!!!!!!!!!!!!!!!
	char fname[PATH_MAX];
	sprintf( fname, "/tmp/%d.fifo", p->pid );

	int fd = open( fname, O_WRONLY );
	if ( -1 == fd ) {
		_LOG_ERR( "cannot open %s: %s", fname, strerror(errno) );
		return -1;
	}

	if( is_valid_message(p) ) {

		p->status = handle_message(p);
		if ( sizeof(message) != write( fd, p, sizeof(message) ) ) {

			_LOG_ERR( "%s: %s", fname, strerror(errno) );
		}
	}
	else _LOG_ERR( "bogus request" );

	VERIFY( 0 == close(fd) );
	return 0;
}

//--------------------------------------------------------------------------

void die( const char *function, int line, const char *what )
{
	_LOG_CRIT( "system error in function `%s': line %d: `%s' -- %s",
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
	"-a ADCPEAK     ADC-rate buffer peak level at 0dBm [ADC count].\n"
	"-d DEC         Decimation factor (sampling freq. to TBT rate).\n"
	"-f fTBT        Revolution frequency [Hz].\n"
	"-g AVG         Averaging in phase compensation [switching periods]\n"	
	"-h             Print this message and exit.\n"
	"-m MDEL        Marker delay [ADC samples].\n"
	"-n NTBT        [TBT samples] per switch position.\n"
	"-p MCPRESC     Machine clock prescaller value for PLL daemon.\n"
	"-r HARMONIC    Harmonic number (RF freq. to TBT rate).\n"
	"-s SUMATT      Sum of attenuators at 0dBm [dB].\n"
	"-t A2DTIME     Analog to digital switch propagation time [ADC samples].\n"
	"-v             Print version information and exit.\n"
	"\n";

	fprintf( stderr, format, argv0 );
}

//--------------------------------------------------------------------------

void version()
{
	const char *format =
	"%s %s (%s %s)\n"
	"\n"
	"Copyright 2006 Instrumentation Technologies.\n"
	"This program is licenced software; you can use it under the terms of the\n"
        "Instrumentation Technologies License. You should have received a copy of the\n"
        "Licence along with this program; if not, write to the Instrumentation\n"
        "Technologies, Velika pot 22, 5250 Solkan, Slovenia.\n";

	printf( format, argv0, XSTR(RELEASE_VERSION), __DATE__, __TIME__ );
}
