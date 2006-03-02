// $Id: ebpp.c,v 1.21 2006/01/12 08:01:51 miha Exp $

//! \file ebpp.c
//! Electron Beam Position Processor (EBPP) specific module.

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

#include "cordic.h"

#include "cspi.h"
#include "cspi_impl.h"

#include "ebpp.h"

/** Max. DD decimation available on the FPGA. */
#define MAX_DEC 64

/** Private. EBPP specific. Local to this module only.
 *
 *  Represents mirrored (cached) environment parameters
 *  used to speed up position calculations.
 */
typedef struct tagCache {
	int Kx; 		//!< Horizontal calibration coefficient.
	int Ky; 		//!< Vertical calibration coefficient.
	int Xoffset;	//!< Electrical/magnetic horizontal offset.
	int Yoffset;	//!< Electrical/magnetic vertical offset.
	int Qoffset;	//!< Electrical offset.
} Cache;

/** Private. EBPP specific. Local to this module only.
 *
 *  One and only Cache instance.
 */
Cache cache;

/** Private. EBPP specific. Local to this module only.
 *
 *  A mutex to protect the cache from concurrent modifications.
 */
pthread_mutex_t cache_mutex = PTHREAD_MUTEX_INITIALIZER;

/** Private. EBPP specific. Local to this module only.
 *
 *  Cache bitmask.
 */
#define CACHE_MASK (CSPI_ENV_KX | CSPI_ENV_KY | \
                    CSPI_ENV_XOFFSET | CSPI_ENV_YOFFSET | CSPI_ENV_QOFFSET)

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Updates cached environment data.
 *  On success, returns CSPI_OK. Otherwise, returns one of the errors
 *  returned by cspi_getenvparam.
 *  @param e Environment handle.
 */
int ebpp_update_cache( Environment *e )
{
	ASSERT(e);
	CSPI_ENVPARAMS ep;

	// Assume environment has been locked by caller.
	ASSERT( EBUSY == pthread_mutex_trylock( &e->mutex ) );

	int rc = custom_getenvparam( e, &ep, CACHE_MASK );
	if ( CSPI_OK ==  rc ) {

		VERIFY( 0 == pthread_mutex_lock( &cache_mutex ) );

		// Assume ep.Kx and ep.Ky are in nanometers (um).
		// No conversion needed!
		cache.Kx = ep.Kx;
		cache.Ky = ep.Ky;

		cache.Xoffset = ep.Xoffset;
		cache.Yoffset = ep.Yoffset;
		cache.Qoffset = ep.Qoffset;

		VERIFY( 0 == pthread_mutex_unlock( &cache_mutex ) );
	}

	return rc;
}

//--------------------------------------------------------------------------

volatile size_t _is_cache_dirty = 0;

void signal_handler_hook( const CSPI_EVENTHDR *p )
{
	if (p->id == CSPI_EVENT_CFG) {

		switch (p->param) {

			case LIBERA_CFG_KX:
			case LIBERA_CFG_KY:
			case LIBERA_CFG_XOFFSET:
			case LIBERA_CFG_YOFFSET:
			case LIBERA_CFG_QOFFSET:
				_is_cache_dirty = 1;
		}
	}
}

//--------------------------------------------------------------------------

int custom_initop()
{
	if (_is_cache_dirty) {

		_is_cache_dirty = 0;

		VERIFY( 0 == pthread_mutex_lock( &environment.mutex ) );
		int rc = ebpp_update_cache( &environment );
		VERIFY( 0 == pthread_mutex_unlock( &environment.mutex ) );

		return rc;
	}

	return CSPI_OK;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates Kx or Ky.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to Kx or Ky.
 */
inline int ebpp_is_validcoef( const void *p )
{
	const int max = 0x3FFFFFFF;	// INT_MAX/4;

	const size_t *K = p;
	return *K <= max;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates Xoffset, Yoffset or Qoffset.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to offset value.
 */
inline int ebpp_is_validoffset( const void *p )
{
	const int min = 0xC0000001;	// INT_MIN/4;
	const int max = 0x3FFFFFFF;	// INT_MAX/4;

	const int offset = *(int *)p;
	return ( offset >= min && offset <= max );
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates decimation factor in DD mode.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to size_t variable with decimation to validate.
 */
inline int ebpp_is_validdec( const void *p )
{
	const size_t dec = *(size_t *)p;
	return ( dec == 1 || dec == MAX_DEC );
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates switch configuration.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to size_t variable with switch configuration to validate.
 */
inline int ebpp_is_validswitch( const void *p )
{
	const size_t *switches =  p;
	return *switches <= 0x0f;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates attenuator settings.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a vector of CSPI_MAXATTN 1-byte values to validate.
 */
inline int ebpp_is_validattn( const void *p )
{
	const size_t *attn = p;
	return *attn <= 0x1f;
}

//--------------------------------------------------------------------------

int custom_initenv( CSPIHANDLE h ) {

	// Call base version first!
	int rc = base_initenv(h);

	// Now do the EBPP specific part.
	_is_cache_dirty = 1;

	return rc;
}

//--------------------------------------------------------------------------

int custom_initcon( CSPIHANDLE h, CSPIHANDLE p ) {

	// Call base version only!
	return base_initcon( h, p );
}

//--------------------------------------------------------------------------

#define CSPI_ENV_ATTN0	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN1	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN2	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN3	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN4	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN5	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN6	CSPI_ENV_ATTN
#define CSPI_ENV_ATTN7	CSPI_ENV_ATTN

int custom_setenvparam( Environment *e, const CSPI_ENVPARAMS *p,
                        CSPI_BITMASK flags )
{
	ASSERT(e);
	ASSERT(p);

	/* Assume environment has been locked by caller. */
	ASSERT( EBUSY == pthread_mutex_trylock( &e->mutex ) );

	// Custom params.
	const Param_map map[] = {

		PARAM( KX, &p->Kx, ebpp_is_validcoef ),
		PARAM( KY, &p->Ky, ebpp_is_validcoef ),

		PARAM( XOFFSET, &p->Xoffset, ebpp_is_validoffset ),
		PARAM( YOFFSET, &p->Yoffset, ebpp_is_validoffset ),
		PARAM( QOFFSET, &p->Qoffset, ebpp_is_validoffset ),

		PARAM( XLOW,  &p->Xlow,  0 ),
		PARAM( XHIGH, &p->Xhigh, 0 ),
		PARAM( YLOW,  &p->Ylow,  0 ),
		PARAM( YHIGH, &p->Yhigh, 0 ),

		PARAM( SWITCH, &p->switches, ebpp_is_validswitch ),

		PARAM( ATTN0, p->attn + 0, ebpp_is_validattn ),
		PARAM( ATTN1, p->attn + 1, ebpp_is_validattn ),
		PARAM( ATTN2, p->attn + 2, ebpp_is_validattn ),
		PARAM( ATTN3, p->attn + 3, ebpp_is_validattn ),
		PARAM( ATTN4, p->attn + 4, ebpp_is_validattn ),
		PARAM( ATTN5, p->attn + 5, ebpp_is_validattn ),
		PARAM( ATTN6, p->attn + 6, ebpp_is_validattn ),
		PARAM( ATTN7, p->attn + 7, ebpp_is_validattn ),

		// Note: must be null terminated!
		{ 0, {0, 0} },
	};

	// Call "base" method to handle common params.
	int rc = base_setenvparam( e, p, flags );
	if ( CSPI_OK == rc ) {

		// Handle ebpp-specific params.
		rc = handle_params( e->fd, (Param_map *)map, flags, SET );
	}

	if ( flags & CACHE_MASK ) _is_cache_dirty = 1;
	return rc;
}

//--------------------------------------------------------------------------

int custom_getenvparam( Environment *e, CSPI_ENVPARAMS *p, CSPI_BITMASK flags )
{
	ASSERT(e);
	ASSERT(p);

	// Assume environment has been locked by caller.
	ASSERT( EBUSY == pthread_mutex_trylock( &e->mutex ) );

	// Custom params
	Param_map map[] = {

		PARAM( KX, &p->Kx, 0 ),
		PARAM( KY, &p->Ky, 0 ),

		PARAM( XOFFSET, &p->Xoffset, 0 ),
		PARAM( YOFFSET, &p->Yoffset, 0 ),
		PARAM( QOFFSET, &p->Qoffset, 0 ),

		PARAM( XLOW,  &p->Xlow,  0 ),
		PARAM( XHIGH, &p->Xhigh, 0 ),
		PARAM( YLOW,  &p->Ylow,  0 ),
		PARAM( YHIGH, &p->Yhigh, 0 ),

		PARAM( SWITCH, &p->switches, 0 ),

		PARAM( ATTN0, p->attn + 0, ebpp_is_validattn ),
		PARAM( ATTN1, p->attn + 1, ebpp_is_validattn ),
		PARAM( ATTN2, p->attn + 2, ebpp_is_validattn ),
		PARAM( ATTN3, p->attn + 3, ebpp_is_validattn ),
		PARAM( ATTN4, p->attn + 4, ebpp_is_validattn ),
		PARAM( ATTN5, p->attn + 5, ebpp_is_validattn ),
		PARAM( ATTN6, p->attn + 6, ebpp_is_validattn ),
		PARAM( ATTN7, p->attn + 7, ebpp_is_validattn ),

		// Note: must be null terminated!
		{ 0, {0, 0} },
	};

	if ( !flags ) return CSPI_OK;

	// Call "base" method to handle common params.
	int rc = base_getenvparam( e, p, flags );
	if ( CSPI_OK == rc ) {

		// Handle ebpp-specific params.
		rc = handle_params( e->fd, map, flags, GET );
	}
	return rc;
}

//--------------------------------------------------------------------------

int custom_setconparam( Connection *con, const CSPI_CONPARAMS *p, CSPI_BITMASK flags )
{
	ASSERT(con);
	ASSERT(p);

	// Call base method to handle common connection params.
	int rc = base_setconparam( con, p, flags );
	if ( CSPI_OK != rc ) return rc;

	// Handle DD connection params, specific to the EBPP.
	if ( flags & CSPI_CON_DEC ) {

		if ( -1 == con->fd ) return CSPI_E_SEQUENCE;	// Connect first!
		if ( CSPI_MODE_DD != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_DD *q = (CSPI_CONPARAMS_DD *)p;
		if ( !ebpp_is_validdec( &q->dec ) ) return CSPI_E_INVALID_PARAM;

		ASSERT(-1 != con->fd);
		if (-1 == ioctl( con->fd, LIBERA_IOC_SET_DEC, &q->dec )) rc = CSPI_E_SYSTEM;
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

	// Handle DD connection params, specific to the EBPP.
	if ( flags & CSPI_CON_DEC ) {

		if ( CSPI_MODE_DD != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_DD *q = (CSPI_CONPARAMS_DD *)p;

		ASSERT(-1 != con->fd);
		if (-1 == ioctl( con->fd, LIBERA_IOC_GET_DEC, &q->dec )) rc = CSPI_E_SYSTEM;
	}

	return rc;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Transforms a CSPI_DD_RAWATOM into CSPI_DD_ATOM. Returns 0.
 *  @param in Pointer to the CSPI_DD_RAWATOM to transform.
 *  @param out Pointer to CSPI_DD_ATOM to overwrite.
 */
int ebpp_rawtosyn( const void *in, void *out )
{
	const CSPI_DD_RAWATOM *p = (CSPI_DD_RAWATOM *)in;
	CSPI_DD_ATOM *q = (CSPI_DD_ATOM *)out;

	const int Va = q->Va = cordic_amp( p->sinVa >> 1, p->cosVa >> 1 );
	const int Vb = q->Vb = cordic_amp( p->sinVb >> 1, p->cosVb >> 1 );
	const int Vc = q->Vc = cordic_amp( p->sinVc >> 1, p->cosVc >> 1 );
	const int Vd = q->Vd = cordic_amp( p->sinVd >> 1, p->cosVd >> 1 );

	const long long int S = (long long)Va+Vb+Vc+Vd;

	long long int X = (long long)Va+Vd-Vb-Vc;
	X *= cache.Kx;
	q->X = (int)(X/S) - cache.Xoffset;

	long long int Y = (long long)Va+Vb-Vc-Vd;
	Y *= cache.Ky;
	q->Y = (int)(Y/S) - cache.Yoffset;

	long long int Q = (long long)Va+Vc-Vb-Vd;
	Q *= cache.Kx;
	q->Q = (int)(Q/S) - cache.Qoffset;

	// Prevent sum overflow
	q->Sum = (int)(S >> 2);

	return 0;
}

//--------------------------------------------------------------------------

CSPI_AUX_FNC custom_getdefaultop( const Connection *p )
{
	ASSERT(p);
	return CSPI_MODE_DD == p->mode ? ebpp_rawtosyn : 0;
}
