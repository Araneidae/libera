// $Id: bbfp.c,v 1.8 2006/01/12 08:01:51 miha Exp $

//! \file bbfp.c
//! Bunch-by-Bunch Feedback Processor (BBFP) specific module.

/*
CSPI - Control System Programming Interface
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
*/

/*
Implementation note: Public functions have their input parameters validated
in both, debug and nondebug builds. Private functions, on the other hand,
validate input parameters in debug build only.

TAB = 4 spaces.
*/

#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "cspi.h"
#include "cspi_impl.h"

#include "bbfp.h"

/** Max. step (the number of bunches to skip). */
#define BBFP_MAXSTEP 992

//--------------------------------------------------------------------------

void signal_handler_hook( const CSPI_EVENTHDR *p ) { /*not used*/ }

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates switch configuration.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to size_t variable with switch configuration to validate.
 */
static inline int bbfp_is_validstep( const void *p )
{
	const size_t *step =  (const size_t *) p;
	return *step <= BBFP_MAXSTEP;
}

//--------------------------------------------------------------------------

int custom_initenv( CSPIHANDLE h )
{
	// Call base version only!
	return base_initenv(h);
}

//--------------------------------------------------------------------------

int custom_initcon( CSPIHANDLE h, CSPIHANDLE p )
{
	// Call base version only!
	return base_initcon( h, p );
}

//--------------------------------------------------------------------------

int custom_setenvparam( Environment *e, const CSPI_ENVPARAMS *p,
                        CSPI_BITMASK flags )
{
	// Call base version only!
	return base_setenvparam( e, p, flags );
}

//--------------------------------------------------------------------------

int custom_getenvparam( Environment *e, CSPI_ENVPARAMS *p, CSPI_BITMASK flags )
{
	// Call base version only!
	return base_getenvparam( e, p, flags );
}

//--------------------------------------------------------------------------

int custom_setconparam( Connection *con, const CSPI_CONPARAMS *p, CSPI_BITMASK flags )
{
	ASSERT(con);
	ASSERT(p);

	// Call base method to handle common connection params.
	int rc = base_setconparam( con, p, flags );
	if ( CSPI_OK != rc ) return rc;

	// Handle DD connection params, specific to the BBFP.
	if ( flags & CSPI_CON_STEP ) {

		if ( -1 == con->fd ) return CSPI_E_SEQUENCE;	// Connect first!
		if ( CSPI_MODE_DD != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_DD *q = (CSPI_CONPARAMS_DD *)p;
		if ( !bbfp_is_validstep( &q->step ) ) return CSPI_E_INVALID_PARAM;

		ASSERT(-1 != con->fd);
		if (-1 == ioctl( con->fd, LIBERA_IOC_SET_DEC, &q->step )) rc = CSPI_E_SYSTEM;
	}

	return rc;
}

//--------------------------------------------------------------------------

int custom_getconparam( Connection *con, CSPI_CONPARAMS *p, CSPI_BITMASK flags )
{
	ASSERT(con);
	ASSERT(p);

	// Call base method to handle common connection params.
	int rc = base_getconparam( con, p, flags );
	if ( CSPI_OK != rc ) return rc;

	// Handle DD connection params, specific to the BBFP.
	if ( flags & CSPI_CON_STEP ) {

		if ( CSPI_MODE_DD != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_DD *q = (CSPI_CONPARAMS_DD *)p;

		ASSERT(-1 != con->fd);
		if (-1 == ioctl( con->fd, LIBERA_IOC_GET_DEC, &q->step )) rc = CSPI_E_SYSTEM;
	}

	return rc;
}

//--------------------------------------------------------------------------

CSPI_AUX_FNC custom_getdefaultop( const Connection *p )
{
	return 0;
}

//--------------------------------------------------------------------------

int custom_initop() { /*not used*/ return CSPI_OK; }
