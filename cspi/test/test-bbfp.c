// $Id: test-bbfp.c,v 1.2 2006/01/12 08:01:51 miha Exp $

// \file test-bbfp.cpp
// Simple utility for data acquisition.

/*
Libera
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

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>

#include "cspi.h"

#define MINARGS 1

// Maximum number of atoms per one read.
#define MAXSIZE ( (32 - 1) * 1024 *1024 )

// Default trigger timeout.
#define DEF_TIMEOUT 30

// Helper macro to stringify the expanded argument.
#define XSTR(s) STR(s)

// Stringification macro.
#define STR(s) #s

// Typedef to represent a byte.
typedef unsigned char byte;

// Trigger condition. Signaled on each TRIGGER trigger.
pthread_cond_t trigger_cond = PTHREAD_COND_INITIALIZER;

// Mutex associated with trigger_cond.
pthread_mutex_t trigger_mutex = PTHREAD_MUTEX_INITIALIZER;

//--------------------------------------------------------------------------

// Print usage information.
void usage();

// Print version information.
void version();

// Print diagnostic message and exit.
void die(int n, ...);

// Return string describing the CSPI error code.
const char *what(int n);

// Write to FAI from stdin.
int write_fai(CSPIHENV henv, size_t size);

// Read from FAI to stdout.
int read_fai(CSPIHENV henv, size_t size);

// Event handler.
int event_callback(CSPI_EVENT *msg);

// Wait for TRIGGER trigger or timeout.
int trigger_timedwait(size_t delay);

// Initialize and connect to a data source.
void init( CSPIHENV *henv, CSPIHCON *hcon);

// CSPI cleanup.
void cleanup( CSPIHENV henv, CSPIHCON hcon );

// Acquire data on trigger.
void acquire(CSPIHCON hcon, size_t count);

// Dump CSPI timestamp to stdout.
int dump_timestamp(CSPIHCON hcon);

//--------------------------------------------------------------------------

// Error codes. See error[] for descriptions.
enum {
	E_INVALID_ARG = 0,
	E_NO_ARG,
	E_SYS,
	E_CSPI,
};

// Error descriptions corresponding to error codes.
const char *error[] = {
	"invalid argument -- %s",
	"missing argument -- %s",
	"system error in function `%s': line %d: %s -- %s",
	"CSPI error in function `%s': line %d: %s",
};

// Helper macro. Print diagnostic system message and exit.
#define DIE(s) die( E_SYS, __FUNCTION__, __LINE__, s, strerror(errno) )

// Helper macro. Print CSPI message and exit if f is non zero.
#define CSPI_WRAP(f) {\
		const int rc=f;\
		if (rc) die( E_CSPI, __FUNCTION__, __LINE__, what(rc) );\
	}

//--------------------------------------------------------------------------

// Bit flags to represent on/off options on the command-line.
typedef enum {

	WANT_BINARY    = 0x01,
	WANT_TIMESTAMP = 0x02,
	WANT_SETSTEP   = 0x04,
	WANT_FAI_READ  = 0x08,
	WANT_FAI_WRITE = 0x10,
}
WANT_FLAGS;

// Program name.
const char *_argv0 = 0;

// Stores a combination of WANT_FLAGS bits.
size_t _want_bits = 0;

// Data retrieval point (relative to trigger) in bunches.
long long _offset = 0;

// Step when reading the data.
size_t _step = 1;

// FAI element size in bytes.
size_t _objsize = 2;

//--------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	_argv0 = argv[0];

	if (argc < MINARGS) {

		usage();
		exit( EXIT_FAILURE );
	}

	const char *optstr = "bho:pr::s:vw::";

	int ch = -1;
	while ( (ch = getopt( argc, argv, optstr )) != -1 ) {
		switch (ch) {

			case 'b':
				_want_bits |= WANT_BINARY;
				break;

			case 'h':
				usage();
				exit(EXIT_SUCCESS);

			case 'o':
				_offset = atoi(optarg);
				break;

			case 'p':
				_want_bits |= WANT_TIMESTAMP;
				break;

			case 'r':
				_want_bits |= WANT_FAI_READ;
				if (optarg) _objsize = atoi(optarg);
				break;

			case 's':
				_want_bits |= WANT_SETSTEP;
				_step = atoi(optarg);
				break;

			case 'v':
				version();
				exit(EXIT_SUCCESS);

			case 'w':
				_want_bits |= WANT_FAI_WRITE;
				if (optarg) _objsize = atoi(optarg);
				break;

			default:
				exit(EXIT_FAILURE);
		}
	}

	if ( optind == argc ) die( E_NO_ARG, "SIZE" );

	const size_t size = atoi( argv[optind] );
	if ( size > MAXSIZE ) die( E_INVALID_ARG, "SIZE" );

	if (_objsize < 2) die( E_INVALID_ARG, "FAI element size" );

	CSPIHENV henv;
	CSPIHCON hcon;

	init(&henv, &hcon);
	if (size) {

		if (_want_bits & WANT_FAI_WRITE) write_fai(henv, size);
		if (_want_bits & WANT_FAI_READ) read_fai(henv, size);

		if ( !(_want_bits & (WANT_FAI_READ|WANT_FAI_WRITE)) )
			acquire(hcon, size);
	}
	cleanup(henv, hcon);

	return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------

void init(CSPIHENV *henv, CSPIHCON *hcon)
{
	CSPI_LIBPARAMS lp;
	lp.superuser = (_want_bits & (WANT_FAI_READ|WANT_FAI_WRITE)) ? 1 : 0;

	CSPI_WRAP( cspi_setlibparam( &lp, CSPI_LIB_SUPERUSER ) );

	CSPI_WRAP( cspi_allochandle( CSPI_HANDLE_ENV, 0, henv ) );
	CSPI_WRAP( cspi_allochandle( CSPI_HANDLE_CON, *henv, hcon ) );

	CSPI_CONPARAMS_DD cp;
	cp.mode = CSPI_MODE_DD;
	cp.handler = event_callback;
	cp.event_mask = CSPI_EVENT_TRIGGET;

	const CSPI_BITMASK mask = CSPI_CON_MODE|CSPI_CON_HANDLER|CSPI_CON_EVENTMASK;

	CSPI_WRAP( cspi_setconparam( *hcon, (CSPI_CONPARAMS *)&cp, mask ));
	CSPI_WRAP( cspi_connect( *hcon ) );

	if (_want_bits & WANT_SETSTEP) {

		cp.step = _step;
		CSPI_WRAP( cspi_setconparam( *hcon, (CSPI_CONPARAMS *)&cp, CSPI_CON_STEP ));
	}
}

//--------------------------------------------------------------------------

void cleanup( CSPIHENV henv, CSPIHCON hcon )
{
	CSPI_WRAP( cspi_disconnect( hcon ) );

	CSPI_WRAP( cspi_freehandle( CSPI_HANDLE_CON, hcon ) );
	CSPI_WRAP( cspi_freehandle( CSPI_HANDLE_ENV, henv ) );
}

//--------------------------------------------------------------------------

int write_fai(CSPIHENV henv, size_t size)
{
	const size_t nbytes = size * _objsize;

	void *buf = malloc(nbytes);
	if (!buf) DIE("malloc");

	fread(buf, _objsize, size, stdout);
	if (ferror(stdin)) DIE("fread");

	CSPI_WRAP( cspi_setenvparam_fa(henv, 0, (int*)buf, _objsize, size) );
	free(buf);

	return 0;
}

//--------------------------------------------------------------------------

int read_fai(CSPIHENV henv, size_t size)
{
	const size_t nbytes = size * _objsize;

	void *buf = malloc(nbytes);
	if (!buf) DIE("malloc");

	CSPI_WRAP( cspi_getenvparam_fa(henv, 0, (int*)buf, _objsize, size) );
	fwrite(buf, _objsize, size, stdout);

	free(buf);
	if (ferror(stdout)) DIE("fwrite");

	return 0;
}

//--------------------------------------------------------------------------

void acquire(CSPIHCON hcon, size_t size)
{
	const size_t atomsize = sizeof(CSPI_DD_ATOM);
	const size_t nbytes = size * atomsize;

	// Allocate buffer large enough to hold raw or synt data.
	CSPI_DD_ATOM *p = (CSPI_DD_ATOM *) malloc(nbytes);
	if (!p) DIE("malloc");

	if (_offset) CSPI_WRAP( cspi_seek(hcon, &_offset, SEEK_CUR) );

	// Wait for trigger ot timeout, whichever happens first.
	if (0 == trigger_timedwait(DEF_TIMEOUT)) {

		errno = ETIMEDOUT;
		DIE( "trigger_timedwait" );
	}

	size_t nread = 0;
	CSPI_WRAP( cspi_read(hcon, p, size, &nread) );

	if (_want_bits & WANT_TIMESTAMP) dump_timestamp(hcon);
	if ( _want_bits & WANT_BINARY ) {

		if (nread != fwrite(p, atomsize, nread, stdout) ) DIE("fwrite");
	}
	else {

		const CSPI_DD_ATOM *q = p;
		for ( size_t i = 0; i < nread; ++i, ++q ) {

			printf("%11d %11d %11d %11d\n",
				q->sample[0], q->sample[1], q->sample[2], q->sample[3] );
		}
	}

	free(p);
}

//--------------------------------------------------------------------------

int dump_timestamp(CSPIHCON hcon)
{
	CSPI_TIMESTAMP ts;
	CSPI_WRAP( cspi_gettimestamp(hcon, &ts) );

	struct tm *p = gmtime( &ts.st.tv_sec );

	// Always use stdout, -f switch should affect data only.
	printf( "MT: %llu, ST: %u-%02u-%02u %02u:%02u:%02u %06ld.%ld UTC\n",
		ts.mt,
		p->tm_year + 1900,
		p->tm_mon + 1,
		p->tm_mday,
		p->tm_hour,
		p->tm_min,
		p->tm_sec,
		ts.st.tv_nsec / 1000,
		ts.st.tv_nsec % 1000 / 100 );

	return 0;
}

//--------------------------------------------------------------------------

volatile int _event_id = 0;

int event_callback(CSPI_EVENT *msg)
{

	_event_id = msg->hdr.id;
	return 1;
}

//--------------------------------------------------------------------------

int trigger_timedwait(size_t delay)
{
	do {
		delay = sleep(delay);
	}
	while (_event_id != CSPI_EVENT_TRIGGET);

	_event_id = 0;
	return delay;
}

//--------------------------------------------------------------------------

const char *what( int n )
{
	static char buf[CSPI_MAX_MSG_LEN];

	if ( CSPI_E_SYSTEM == n ) {

		memset( buf, 0, sizeof(buf) );
		sprintf( buf, "%s: %s (%d)", cspi_strerror(n), strerror(errno), errno );
		return buf;
	}

	return cspi_strerror(n);
}

//--------------------------------------------------------------------------

void die( int n, ... )
{
	fprintf( stderr, "%s: ", _argv0 );
	
	va_list ap;

	va_start( ap, n );
	vfprintf( stderr, error[n], ap );
	va_end( ap );

	fputc( '\n', stderr );
	exit( EXIT_FAILURE );
}

//--------------------------------------------------------------------------

void usage()
{
	const char *format =
	"Usage: %s [OPTION]... SIZE\n"
	"\n"
	"-b              Output bunches as binary data (default: format as text).\n"
	"-h              Print this message and exit.\n"
	"-r [nbytes]     Read 'SIZE' elements of data, each 'nbytes' long, from\n"
	"                FA Interface to stdout. Unless specified, nbytes=2.\n"
	"-w [nbytes]     Write 'SIZE' elements of data, each 'nbytes' long, to\n"
	"                FA Interface from stdin. Unless specified, nbytes=2.\n"
	"-o offset       Reposition the data retrieval point to argument 'offset',\n"
	"                relative to trigger. (default: bunch 0).\n"
	"-p              Print a timestamp of the first bunch.\n"
	"-s step         Set the step in bunches when reading the data.\n"
	"-v              Print version information and exit.\n"
	"SIZE            The number of bunches to acquire to stdout.\n";

	fprintf(stderr, format, _argv0);
}

//--------------------------------------------------------------------------

void version()
{
	const char *format =
	"%s %s (%s %s)\n"
	"\n"
	"Copyright 2005 Instrumentation Technologies.\n"
	"This is free software; see the source for copying conditions. "
	"There is NO warranty; not even for MERCHANTABILITY or FITNESS "
	"FOR A PARTICULAR PURPOSE.\n";

	printf( format, _argv0, XSTR(RELEASE_VERSION), __DATE__, __TIME__ );
}
