// $Id: acquire.c,v 1.28 2006/01/12 08:01:51 miha Exp $

// \file acquire.c
// Utility for Data-on-Demand acquisition.

/*
Acquire
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

#if defined(CLIENT_SERVER)
#include "client-lib.h"
#define MAXCACHE  (16*1024)
#endif	// CLIENT_SERVER

#define MINARGS 2

// Maximum number of atoms per one read.
#define MAXSIZE ( (65536 - 1) * 32 )

// Maximum pathname length in characters.
#define MAXPATH 255

// Default decimation.
#define DEF_DECIMATION 1

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
void die( int n, ... );

// Return string describing CSPI error code.
const char *what( int n );

// Get attenuator values from a string (i.e. a command-line argument).
int get_attn( CSPI_ENVPARAMS *ep, const char *arg );

// Event handler.
int trigger_callback( CSPI_EVENT *msg );

// Wait for TRIGGER trigger condition.
int trigger_timedwait( size_t delay );

// Initialize CSPI and connects to a data source.
void init( CSPIHENV *henv, CSPI_ENVPARAMS *ep, size_t ef,
           CSPIHCON *hcon, CSPI_CONPARAMS_DD *cp, size_t cf  );

// Carry out CSPI cleanup.
void cleanup( CSPIHENV henv, CSPIHCON hcon );

// Acquire data from history buffer.
void acquire( CSPIHCON hcon, size_t count, size_t repeat );

//--------------------------------------------------------------------------

// Error codes. See error[] for descriptions.
enum {
	E_INVALID_ARG = 0,
	E_NO_ARG,
	E_SYS,
	E_CSPI,
	E_SERVER,
};

// Error descriptions corresponding to error codes.
const char *error[] = {
	"invalid argument -- %s",
	"missing argument -- %s",
	"system error in function `%s': line %d: %s -- %s",
	"CSPI error in function `%s': line %d: %s",
	"SERVER error in function `%s': line %d: %s",
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

	WANT_RAWDATA   = 0x01,
	WANT_TIMESTAMP = 0x02,
	WANT_BINARY    = 0x04,
}
WANT_FLAGS;

// Stores a combination of WANT_FLAGS bits.
size_t _want_bits = 0;

// Type of the data retrieval point (MT or TRIGGER).
int _type = CSPI_SEEK_MT;

// Data retrieval point (history buffer offset) in MT.
unsigned long long _offset = 0;

// Output filename.
char _filename[ MAXPATH ];

// Program name.
const char *_argv0 = 0;

// Multicast address.
const char *mcast_addr = 0;

// Cache size.
size_t cache_size = 0;

// Acquire repetititons.
size_t acq_repeat = 1;

// Acquire infinite repetititons flag.
size_t inf_repeat = 0;

//--------------------------------------------------------------------------

int main( int argc, char *argv[] )
{
	_argv0 = argv[0];

	if ( argc < MINARGS ) {

		usage();
		exit( EXIT_FAILURE );
	}

	CSPI_ENVPARAMS ep;
	CSPI_CONPARAMS_DD cp;

	ep.trig_mode = CSPI_TRIGMODE_GET;
	size_t ef = CSPI_ENV_TRIGMODE;

	cp.mode = CSPI_MODE_DD;
	size_t cf = CSPI_CON_MODE;

	// File name stem used to construct the output file name.
	const char *fname = 0;

#if defined(CLIENT_SERVER)
	const char *optstring = "a:bc:d:f:hm:n::o:prs:tv";
#else
	const char *optstring = "a:bd:f:hn::o:prs:tv";
#endif	// CLIENT_SERVER

	int ch = -1;
	while ( (ch = getopt( argc, argv, optstring )) != -1 )
	{
		switch ( ch ) {

			case 'a':
				if ( get_attn( &ep, optarg ) != CSPI_MAXATTN ) die( E_INVALID_ARG, "'a'" );
				ef |= CSPI_ENV_ATTN;
				break;

			case 'b':
				_want_bits |= WANT_BINARY;
				break;

#if defined(CLIENT_SERVER)
			case 'c':
				if ( MAXCACHE < atol( optarg ) ) die( E_INVALID_ARG, "'c'" );
				cache_size = atol( optarg );
				 // option -c implies -r
				if ( 0 != cache_size ) _want_bits |= WANT_RAWDATA;
				break;

			case 'm':
				mcast_addr = optarg;
				break;
#endif	// CLIENT_SERVER

			case 'd':
				if ( 64 != atol( optarg ) ) die( E_INVALID_ARG, "'d'" );
				cf |= CSPI_CON_DEC;
				cp.dec = 64;
				break;

			case 't':
				_type = CSPI_SEEK_TR;
				break;

			case 'f':
				fname = optarg;
				break;

			case 'h':
				usage();
				exit( EXIT_SUCCESS );

			case 'n':
				if (optarg)
					acq_repeat = atol( optarg );
				else
					inf_repeat = 1;
				break;

			case 'o':
				_offset = atoll( optarg );
				break;

			case 'p':
				_want_bits |= WANT_TIMESTAMP;
				break;

			case 'r':
				_want_bits |= WANT_RAWDATA;
				break;

			case 's':
				ef |= CSPI_ENV_SWITCH;
				ep.switches = atoi( optarg );
				if ( ep.switches > 0xF ) die( E_INVALID_ARG, "'s'" );
				break;

			case 'v':
				version();
				exit( EXIT_SUCCESS );

			default:
				exit( EXIT_FAILURE );
		}
	}

#if defined(CLIENT_SERVER)
	if ( optind == argc ) die( E_NO_ARG, "IP_ADDRESS" );
	const char *addr = argv[optind++];

	if ( optind == argc ) die( E_NO_ARG, "PORT" );
	const int port = atoi( argv[optind++] );
#endif	// CLIENT_SERVER

	if ( optind == argc ) die( E_NO_ARG, "SIZE" );

	const size_t size = atoi( argv[ optind ] );
	if ( size > MAXSIZE ) die( E_INVALID_ARG, "SIZE" );

	memset( _filename, 0, sizeof( _filename ) );
	if ( fname )
		sprintf( _filename, "%s_P%d.raw", fname, ep.switches );

#if defined(CLIENT_SERVER)
	if ( (0 != cache_size) && ( size > cache_size ) ) die( E_INVALID_ARG, "SIZE" );

	if ( 0 != server_connect( addr, port, mcast_addr, 0 ) ) DIE( "server_connect" );
	if ( 0 != server_setparam( SERVER_CACHE_SIZE, &cache_size )) DIE( "server_setparam" );
#endif	// CLIENT_SERVER

	CSPIHENV henv; CSPIHCON hcon;
	init( &henv, &ep, ef, &hcon, &cp, cf );

	if ( size > 0 ) acquire( hcon, size , acq_repeat);
	cleanup( henv, hcon );

#if defined(CLIENT_SERVER)
	if ( 0 != server_disconnect() ) DIE( "server_disconnect" );
#endif	// CLIENT_SERVER

	return EXIT_SUCCESS;
}

//--------------------------------------------------------------------------

void init( CSPIHENV *henv, CSPI_ENVPARAMS *ep, size_t ef,
           CSPIHCON *hcon, CSPI_CONPARAMS_DD *cp, size_t cf )
{
	CSPI_LIBPARAMS lp;
	lp.superuser = 1;

	CSPI_WRAP( cspi_setlibparam( &lp, CSPI_LIB_SUPERUSER ) );

	CSPI_WRAP( cspi_allochandle( CSPI_HANDLE_ENV, 0, henv ) );
	CSPI_WRAP( cspi_setenvparam( *henv, ep, ef ) );

	CSPI_WRAP( cspi_allochandle( CSPI_HANDLE_CON, *henv, hcon ) );

	// Register callback function for triggered acquisition.
	if ( CSPI_SEEK_TR == _type ) {

		((CSPI_CONPARAMS *)cp)->handler = trigger_callback;
		cf |= CSPI_CON_HANDLER;
	}

	CSPI_WRAP( cspi_setconparam( *hcon, (CSPI_CONPARAMS *)cp, cf & ~CSPI_CON_DEC ));
	CSPI_WRAP( cspi_connect( *hcon ) );

	if ( cf & CSPI_CON_DEC )
		CSPI_WRAP( cspi_setconparam( *hcon, (CSPI_CONPARAMS *)cp, CSPI_CON_DEC ));
}

//--------------------------------------------------------------------------

void cleanup( CSPIHENV henv, CSPIHCON hcon )
{
	CSPI_WRAP( cspi_disconnect( hcon ) );

	CSPI_WRAP( cspi_freehandle( CSPI_HANDLE_CON, hcon ) );
	CSPI_WRAP( cspi_freehandle( CSPI_HANDLE_ENV, henv ) );
}

//--------------------------------------------------------------------------

void acquire( CSPIHCON hcon, size_t count, size_t repeat )
{
	FILE *fp = 0;
	if ( *_filename ) {

		fp = fopen( _filename, "w" );
		if ( 0 == fp ) DIE( _filename );
	}
	else fp = fdopen( 1, "w" );	// Use stdout.

	// Since sizeof(CSPI_DD_ATOM) == sizeof(CSPI_DD_RAWATOM),
	// we treat raw data as if it was of type CSPI_DD_ATOM!
	size_t size = count * sizeof( CSPI_DD_ATOM );

	// Allocate buffer large enough to hold raw or synt data.
	CSPI_DD_ATOM *p = (CSPI_DD_ATOM *) malloc( size );
	if ( !p ) DIE( "malloc" );

	/* Acquire data in a loop */
	for (size_t j = 0; ((j < repeat) || inf_repeat); ++j) {

		CSPI_WRAP( cspi_seek( hcon, &_offset, _type ) );
	
		if ( CSPI_SEEK_TR == _type && 0 != trigger_timedwait(DEF_TIMEOUT) ) {
	
			errno = ETIMEDOUT;
			DIE( "trigger_timedwait" );
		}

#if defined(CLIENT_SERVER)
		// Reading cached data? -> Check cache lock status.
		if ( 0 != cache_size ) {
			int val;
			if ( 0 != server_getparam( SERVER_CACHE_LOCK, &val ) ) DIE( "server_getparam" );
			if ( 0 == val )
				die( E_SERVER, __FUNCTION__, __LINE__, "Cache unlocked." );
		}
#endif	// CLIENT_SERVER	

	size_t nread = 0;
	if ( _want_bits & WANT_RAWDATA ) {

		CSPI_WRAP( cspi_read_ex( hcon, p, count, &nread, 0 ) );
	}
	else {

		CSPI_WRAP( cspi_read( hcon, p, count, &nread ) );
	}

	if ( _want_bits & WANT_TIMESTAMP ) {

		CSPI_TIMESTAMP ts;
		CSPI_WRAP( cspi_gettimestamp( hcon, &ts ) );

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
	}

	if ( _want_bits & WANT_BINARY ) {

		if ( count != fwrite( p, sizeof(CSPI_DD_ATOM), count, fp ) )
			DIE("fwrite");
	}
	else {

		for ( size_t i = 0; i < count; ++i ) {
	
			fprintf( fp, "%11d %11d %11d %11d %11d %11d %11d %11d\n",
					(p + i)->Va,
					(p + i)->Vb,
					(p + i)->Vc,
					(p + i)->Vd,
					(p + i)->X,
					(p + i)->Y,
					(p + i)->Q,
					(p + i)->Sum
			);
		}
	}

#if defined(CLIENT_SERVER)
		if ( ( 0 != cache_size ) && ( CSPI_SEEK_TR == _type ) ) {
		    int val = 0;
			if (  0 != server_setparam( SERVER_CACHE_LOCK, &val )) DIE( "server_setparam" );
		}
#endif	// CLIENT_SERVER
	} // for()

	// TODO: Proper cleanup via signal handler.
	free( p );
	if ( *_filename ) fclose( fp );
}

//--------------------------------------------------------------------------

int trigger_callback( CSPI_EVENT *msg )
{
	if ( CSPI_EVENT_TRIGGET == msg->hdr.id ) {

		pthread_cond_signal( &trigger_cond );
		return 0;
	}

	return 1;
}

//--------------------------------------------------------------------------

int trigger_timedwait( size_t delay )
{
	struct timeval  now;
	struct timespec timeout;

	gettimeofday( &now, 0 );

	// Offset timeout 'delay' seconds in the future.
	timeout.tv_sec = now.tv_sec + delay;
	timeout.tv_nsec = now.tv_usec * 1000;

	int rc = 0;
	pthread_mutex_lock( &trigger_mutex );

	do {
		rc = pthread_cond_timedwait( &trigger_cond,
	                                 &trigger_mutex,
	                                 &timeout );
	} while ( EINTR == rc );

	pthread_mutex_unlock( &trigger_mutex );
	return rc;
}

//--------------------------------------------------------------------------

int get_attn( CSPI_ENVPARAMS *ep, const char *arg )
{
	const char delimiters[] = " ,:;\t";

	// Make a private copy of arg so it can be modified in-place.
	char buf[ MAXPATH ];
	
	strncpy( buf, arg, MAXPATH );
	buf[ MAXPATH - 1 ] = 0;

	int i = 0;
	const char *p = strtok( buf, delimiters );
	int *vals = ep->attn;
	
	while ( p && i < CSPI_MAXATTN ) {

		vals[i] = atoi( p );
		if ( vals[i] > 0x1F ) break;

		i += 1;
		p = strtok( 0, delimiters );
	}

	return i;
}

//--------------------------------------------------------------------------

const char *what( int n )
{
	static char buf[ MAXPATH ];

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
	"Usage: %s [OPTION]... %s\n"
	"\n"
	"-a attenuators  A whitespace, comma, colon or semicolon separated\n"
	"                list of eight attenuator values. (default: use current).\n"
	"-b              Binary output (default: formatted text output).\n"
#if defined(CLIENT_SERVER)
	"-c size         Use cache of size \"size\" (default: 0; cache disabled).\n"
	"                This options implies -r, since only raw I and Q data can be\n"
	"                cached.\n"
#endif
	"-f filename     Redirect output to a file (default: stdout).\n"
	"-h              Print this message and exit.\n"
	"-d decimation   Post filtering factor (default: %d).\n"
#if defined(CLIENT_SERVER)
	"-m address      Multicast group address to join (default: 224.0.1.240 ).\n"
#endif
	"-n [count]      Acquire in a loop \"count\" times (default: 1). Omitting\n"
	"                \"count\" will result in an infinite loop.\n"
	"-o offset       Data retrieval point in machine time (default: cur. time).\n"
	"-p              Print a timestamp of the first sample.\n"
	"-r              Acquire raw I and Q data (default: synthetic data --\n"
	"                aplitudes and positions).\n"
	"-s switch       Analog switch configuration (default: use current).\n"
	"-t              Acquire data on trigger (default: off).\n"
	"-v              Print version information and exit.\n"
	"SIZE            The number of samples to acquire.\n"
	"\n"
	"Example: %s -a 0,0,0,0,0,0,0,0 -s 3 %s\n";

#if defined(CLIENT_SERVER)
	const char *args1 = "IP_ADDRESS PORT SIZE";
	const char *args2 = "127.0.0.1 23271 8192";
#else
	const char *args1 = "SIZE";
	const char *args2 = "8192";
#endif

	fprintf( stderr, format, _argv0, args1, DEF_DECIMATION, _argv0, args2 );
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
	"FOR A PARTICULAR PURPOSE.\n";

	printf( format, _argv0, XSTR(RELEASE_VERSION), __DATE__, __TIME__ );
}
