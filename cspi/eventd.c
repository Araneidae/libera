// $Id: eventd.c,v 1.14 2006/07/03 10:43:59 ales Exp $

//! \file eventd.c
//! Implements CSPI Event Daemon.

/*
CSPI Event Daemon
Copyright (C) 2004-2006 Instrumentation Technologies

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
#include "eventd.h"

/** Min. number of arguments taken by the application. */
#define MIN_ARGS 1

/** Max. number of arguments taken by the application. */
#define MAX_ARGS 3

/** Libera initial event mask. */
#define INIT_MASK ( LIBERA_EVENT_PM | LIBERA_EVENT_INTERLOCK )

/** Helper macro to stringify the expanded argument. */
#define XSTR(s) STR(s)

/** Stringification macro. */
#define STR(s) #s

/** Helper macro. Print diagnostic system message and exit. */
#define EXIT(what) die( __FUNCTION__, __LINE__, what )

/** Helper macro. Return larger of a and b. */
#define MAX(a,b) ((a)>(b) ? a : b)

/** Helper macro. Return lesser of a and b. */
#define MIN(a,b) ((a)<(b) ? a : b)

/** Defines symbolic names for read and write end of a pipe. */
enum {
	RD = 0,
	WR = 1
};

//--------------------------------------------------------------------------
// Globals.

/** Pointer to the application file name. */
const char *argv0 = 0;

/** List of event listeners. */
Listener *listener_head = 0;

/** Size of the event listener list. */
size_t listener_count = 0;

/** Libera device file descriptor. */
int event_fd = -1;

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
 *  Listen to client requests on a named pipe (fifo) and
 *  async events on event pipe. On new client request, add/remove
 *  client from the client list. On new event, dispatche event to
 *  the client list.
 *  Returns 0.
 */
int run();

/** Read up to ntotal bytes from a file descriptor.
 *  On a subsequent call(s), attempts to read the missing bytes.
 *  On success, returns the number of bytes read.
 *  On error, returns -1 (errno may be set to any of the errors
 *  specified for the routine read).
 *  The nleft argument is set to the number of bytes left to read.
 *  @param fd File descriptor.
 *  @param buf Pointer to destination buffer.
 *  @param count The number of bytes to read.
 *  @param nleft The number of left to be read.
 */
int readsome( int fd, void *buf, const size_t ntotal, size_t *nleft );

/** Handle a request to join or leave the listener group.
 *  On success, returns 0. On error, returns -1
 *  (errno is set appropriately).
 *  @param p Pointer to request structure.
 */
int handle_request( const Request *p  );

/** Dispatch event to all listeners.
 *  Returns 0.
 *  @param p Pointer to event structure.
 */
int handle_event( const libera_event_t *p  );

/** Find if a process exists.
 *  Returns 1 (exists) or 0 (does not exist).
 *  @param fname Pointer to pid filename.
 */
int find_instance( const char *fname );

/** Insert a process into the event listener list.
 *  On success, returns a pointer to new list item. On error,
 *  returns -1 (the errno is set appropriately).
 *  @param pid Process id to insert.
 *  @param mask Event mask.
 */
Listener* insert_listener( pid_t pid, size_t mask );

/** Remove a process from the event listener list.
 *  @param pid Process id to remove.
 */
void remove_listener( Listener *p );

/** Find a process in the event listener list.
 *  On success, returns a pointer to the list item or 0
 *  if not found.
 *  @param pid Process id to find.
 */
Listener* find_listener( pid_t pid );

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
	while ( (ch = getopt( argc, argv, "hv" )) != -1 )
	{
		switch ( ch ) {

			case 'h':
				usage();
				exit( EXIT_SUCCESS );
			
			case 'v':
				version();
				exit( EXIT_SUCCESS );

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
	sa.sa_handler = signal_handler;

	// Handle Ctrl-C and regular termination requests.
	const int sigs[] = { SIGINT, SIGHUP, SIGTERM, SIGQUIT };

	for (int i=0; i<sizeof(sigs)/sizeof(int); ++i) {

		if ( 0 != sigaction( sigs[i], &sa, 0 ) ) EXIT( "sigaction" );
	}

	umask(0);
	// It is not an error if the request fifo already exists.
	if ( 0 != mkfifo( EVENTD_REQ_FIFO_PATHNAME, 0666 ) && EEXIST != errno) {

		EXIT( "mkfifo" );
	}

	if ( 0 != find_instance( EVENTD_PID_PATHNAME ) ) {

		_LOG_ERR( "cannot run more than one daemon instance" );
		exit( EXIT_FAILURE );
	}

	// Open Libera event device in RW mode to gain exclusive
	// access to the event fifo. This is also more effective
	// than the check above and makes it kind of redundant!
	event_fd = open( LIBERA_EVENT_FIFO_PATHNAME, O_RDWR );
	if( -1 == event_fd ) EXIT( "open" );

	size_t mask = INIT_MASK;
	if ( 0 != ioctl( event_fd, LIBERA_EVENT_SET_MASK, &mask ) ) EXIT("ioctl");

	// Finally, create a pid file.
	FILE *fp = fopen( EVENTD_PID_PATHNAME, "w" );
	if (!fp) EXIT( "fopen" );

	fprintf( fp, "%d\n", getpid() );
	if ( 0 != fclose(fp) ) EXIT( "fclose" );

	_LOG_DEBUG( "created pid file %s", EVENTD_PID_PATHNAME );
	return 0;
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
	// Disable event fifo
    size_t mask = 0;
	if ( 0 != ioctl( event_fd, LIBERA_EVENT_SET_MASK, &mask ) )
		_LOG_CRIT( "cannot reset event mask" );

	// Release the listener list.
	while (listener_head) remove_listener( listener_head );

	// Remove pid file
	if ( 0 != unlink( EVENTD_PID_PATHNAME ) ) {

		_LOG_ERR( "failed to unlink %s: %s",
		          EVENTD_PID_PATHNAME,
		          strerror( errno ) );
		return;
	}

	_LOG_DEBUG( "removed PID file %s", EVENTD_PID_PATHNAME );
}

//--------------------------------------------------------------------------

int run()
{
	// Open request fifo in non-blocking mode to prevent open 
	// to block if the other end not open.
	int request_fd = open( EVENTD_REQ_FIFO_PATHNAME, O_RDONLY|O_NONBLOCK );
	if ( -1 == request_fd ) EXIT( "open" );

	// Do not use a _LOG_NOTICE macro!
	// We want the following logged regardless of the current log level.
	syslog( LOG_NOTICE,
	        "%s %s configured -- resuming normal operations",
	        argv0,
	        XSTR(RELEASE_VERSION) );

	fd_set rfds;
	const int max_fd = MAX( event_fd, request_fd );

	size_t nleft1 = 0, nleft2 = 0;
	while (1) {

		FD_ZERO( &rfds );

		// Add request fifo descriptor and event pipe descriptors
		// to the set of descriptors to watch.
		FD_SET( event_fd, &rfds );
		FD_SET( request_fd, &rfds );

		// Watch the descriptor set for available input.
		int rc = select( max_fd + 1, &rfds, 0, 0, 0 );
		if ( -1 == rc ) {

			if ( EINTR != errno ) EXIT( "select" );
			continue;
		}

		ssize_t nread = -1;
		if ( FD_ISSET( event_fd, &rfds ) ) {

			libera_event_t event;

			nread = readsome( event_fd, &event, sizeof(event), &nleft1 );
			if ( -1 == nread && EINTR != errno ) EXIT( "readsome" );

			if ( 0 == nleft1 ) handle_event( &event );
		}
		if ( FD_ISSET( request_fd, &rfds ) ) {

			Request req;

			nread = readsome( request_fd, &req, sizeof(req), &nleft2 );
			if ( -1 == nread && EINTR != errno ) EXIT( "readsome" );

			if ( 0 == nleft2 && handle_request( &req ) ) 
			 	_LOG_ERR( "cannot handle request: %s", strerror(errno) );

			// Re-open fifo if closed by peer.
			if ( 0 == nread ) {

				nleft2 = 0;	// discard any incomplete request
				if (-1 == close(request_fd) ) EXIT("close");

				request_fd = open( EVENTD_REQ_FIFO_PATHNAME, O_RDONLY|O_NONBLOCK );
				if ( -1 == request_fd ) EXIT( "open" );
			}
		}
	}

	return 0;
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

int handle_request( const Request *p )
{
	ASSERT(p);

	const int no_signal = 0;
	// Test if pid exists; ignore EPERM permission error.
	if( 0 != kill( p->pid, no_signal ) && EPERM != errno ) return -1;

	Listener *q = find_listener( p->pid );
	if (p->mask) {

		// Attempt to JOIN the list...
		if (q) q->mask = p->mask;	// update mask only
		else if ( 0 == insert_listener( p->pid, p->mask ) ) EXIT( "insert_listener" );
	}
	else {

		// Attempt to LEAVE the list...
		if (q) remove_listener(q);
		else _LOG_WARNING( "pid %d is not a listener", p->pid );
	}

	size_t mask = INIT_MASK;
	for (const Listener* p=listener_head; p; p=p->next) mask |= p->mask;

	return ioctl( event_fd, LIBERA_EVENT_SET_MASK, &mask );
}

//--------------------------------------------------------------------------

int handle_event( const libera_event_t *p )
{
	ASSERT(p);

	// Will block until PM data buffered.
	if (LIBERA_EVENT_PM==p->id && -1==ioctl(event_fd, LIBERA_EVENT_ACQ_PM))
		_LOG_CRIT( "PM event not acknowledged -- %s", strerror(errno) );

	// We can only pass a 32-bit value to a listener process with
	// sigqueue and libera_event_t contains two 32-bit values.
	// Hack -- we assume both values are less than 2^16 and pack
	// them into one 32-bit value.

	ASSERT( p->id < 0xffff );
	ASSERT( p->param < 0xffff );

	sigval_t val;
	val.sival_int = (p->id << 16) | (0x0000ffff & p->param);

	Listener *q = listener_head, *ghost = 0;

	while (q) {

		if ((p->id & q->mask) &&
		     0 != sigqueue(q->pid, LIBERA_SIGNAL, val)) {

			if (errno == ESRCH) ghost = q;	// no matching pid
			_LOG_ERR( "cannot send signal to process %d -- %s", q->pid, strerror(errno) );
		}

		q = q->next;
		if (ghost) {

			remove_listener( ghost );
			ghost = 0;
		}
	}

	return 0;
}

//--------------------------------------------------------------------------

Listener* insert_listener( pid_t pid, size_t mask )
{
	Listener *p = (Listener *) malloc( sizeof(Listener) );
	if (!p) return 0;

	p->pid = pid;
	p->mask = mask;
	p->next = p->prev = 0;

	if (listener_head) {

		p->next = listener_head;
		p->next->prev = p;
	}

	listener_head = p;
	++listener_count;
	
	return p;
}

//--------------------------------------------------------------------------

void remove_listener( Listener* p )
{
	ASSERT(p);

	if (p) {

		if ( p->prev ) p->prev->next = p->next;
		if ( p->next ) p->next->prev = p->prev;

		if ( listener_head == p ) listener_head = p->next;
		p->prev = p->next = 0;

		--listener_count;
		free(p);
	}
}

//--------------------------------------------------------------------------

Listener* find_listener( pid_t pid )
{
	Listener *p = listener_head;
	while( p && p->pid != pid ) p = p->next;

	return p;
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
	"-h              Print this message and exit.\n"
	"-v              Print version information and exit.\n"
	"\n";

	fprintf( stderr, format, argv0 );
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
