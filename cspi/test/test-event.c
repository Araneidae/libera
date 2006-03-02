#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

#include "libera.h"
#include "cspi.h"

const char *_ARG0 = "";
size_t _mask = 0x00;

enum {RD=0, WR=1};
int _fd[2];

void terminate( int err, const char *errstr )
{
	fprintf( stderr, "%s: %s: %s", _ARG0, errstr, cspi_strerror(err) );
	if ( CSPI_E_SYSTEM == err ) {

		fprintf( stderr, ": %s", strerror(errno) );
	}

	fputc('\n', stderr);
	exit(1);
}

//--------------------------------------------------------------------------

int event_callback( CSPI_EVENT *msg )
{
	if (-1 == write(_fd[WR], &(msg->hdr), sizeof(msg->hdr))) perror("write");
	return 1;
}
//--------------------------------------------------------------------------

int dump_event(const CSPI_EVENTHDR *p)
{
	char *s1 = "?", *s2 = "-";

	switch (p->id) {

		case CSPI_EVENT_TRIGGET:
			s1 = "TRIGGET";
		break;

		case CSPI_EVENT_TRIGSET:
			s1 = "TRIGSET";
		break;

		case CSPI_EVENT_CFG:
			s1 = "CFG";
		break;

		case CSPI_EVENT_FA:
			s1 = "FA";
			switch (p->param){

				case CSPI_TRIG_FA_MC0:
					s2 = "MC0";
					break;

				case CSPI_TRIG_FA_MC1:
					s2 = "MC1";
					break;

				case CSPI_TRIG_FA_SC0:
					s2 = "SC0";
					break;

				case CSPI_TRIG_FA_SC1:
					s2 = "SC1";
					break;
			}
		break;

		case CSPI_EVENT_USER:
			s1 = "USER";
			break;

		case CSPI_EVENT_SA:
			s1 = "SA";
		break;

		case CSPI_EVENT_PM:
			s1 = "PM";
		break;

		case CSPI_EVENT_INTERLOCK:
			s1 = "INTERLOCK";
		break;

		case CSPI_EVENT_OVERFLOW:
			s1 = "OVERFLOW";
			switch (p->param){

				case CSPI_OVERFLOW_DD_FPGA:
					s2 = "DD_FPGA";
					break;
	
				case CSPI_OVERFLOW_SA_FPGA:
					s2 = "SA_FPGA";
					break;
	
				case CSPI_OVERFLOW_SA_DRV:
					s2 = "SA_DRV";
					break;
			}
		break;
	}

	printf("id = %d (%s), param = %d (%s)\n", p->id, s1, p->param, s2 );
	return 0;
}

//--------------------------------------------------------------------------

int ctor(CSPIHENV *henv, CSPIHCON *hcon)
{
	CSPI_LIBPARAMS lp;
	lp.superuser = 0;

	int rc = cspi_setlibparam( &lp, CSPI_LIB_SUPERUSER );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_setlibparam" );

	rc= cspi_allochandle( CSPI_HANDLE_ENV, 0, henv );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_allochandle" );

	rc= cspi_allochandle( CSPI_HANDLE_CON, *henv, hcon );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_allochandle" );

	CSPI_CONPARAMS cp;
	cp.handler = event_callback;
	cp.event_mask = _mask;

	rc = cspi_setconparam( *hcon, &cp, CSPI_CON_HANDLER|CSPI_CON_EVENTMASK );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_setlibparam" );

	return 0;
}

//--------------------------------------------------------------------------

int dtor(CSPIHENV henv, CSPIHCON hcon)
{
	int rc = cspi_freehandle( CSPI_HANDLE_CON, hcon );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_freehandle" );

	rc = cspi_freehandle( CSPI_HANDLE_ENV, henv );
	if ( CSPI_OK != rc ) terminate( rc, "cspi_freehandle" );

	return 0;
}

//--------------------------------------------------------------------------

int test(CSPIHENV henv)
{
	CSPI_EVENTHDR msg;

	while (1) {

		if (sleep(30)) {
			if (-1==read(_fd[RD], &msg, sizeof(msg))) {
				perror("read");
				break;
			}
			dump_event(&msg);
		}
		else {
			fprintf(stderr, "No event within 30 seconds.\n");
			break;
		}
	}

	return 0;
}

//--------------------------------------------------------------------------

int main( int argc, char *argv[] )
{
	_ARG0 = argv[0];

	if ( argc != 2 ) {
	
		printf( "usage: %s mask\n", _ARG0 );
		exit(1);
	}
	_mask = strtol(argv[1], 0, 0);

	if (-1 == pipe(_fd)) {

		perror("pipe");
		exit(1);
	}

	CSPIHENV henv;
	CSPIHCON hcon;

	ctor(&henv, &hcon);
	test(henv);

	dtor(henv, hcon);
	return 0;
}
