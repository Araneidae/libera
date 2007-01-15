// $Id: ebpp.c,v 1.45 2006/11/21 12:22:17 ales Exp $

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
#define _GNU_SOURCE
#include <stdio.h>	// sprintf, getline
#include <ctype.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/limits.h>	// PATH_MAX

#include "cordic.h"

#include "cspi.h"
#include "cspi_impl.h"

#include "dscd.h"
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
 *  Update cached environment data.
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

/** Private. EBPP specific. Local to this module only.
 *
 *  Send a message to  the DSC server.
 *  Returns server reply status (typically CSPI_OK on success)
 *  or one of the following errors:
 *  CSPI_E_SYSTEM,
 *  CSPI_E_DSCPROTO.
 *
 *  @param msg_type Message type.
 *  @param msg_val  Pointer to msg_type specific value.
 */
int ebpp_dsc_message( size_t msg_type, int *msg_val )
{
	int rc = CSPI_E_SYSTEM;
	const pid_t pid = getpid();

	char fname[PATH_MAX];
	sprintf( fname, "/tmp/%d.fifo", pid );

	if ( mkfifo( fname, 0600 ) && EEXIST != errno ) return CSPI_E_SYSTEM;

	int srv_fd = open( DSCD_FIFO_PATHNAME, O_WRONLY );
	if ( -1 == srv_fd ) goto cleanup;

	message msg = { DSCD_MAGIC, msg_type, *msg_val, pid, 0 };
	ssize_t n = write( srv_fd, &msg, sizeof(message) );

	close( srv_fd );
	if ( n != sizeof(message) ) goto cleanup;

	int fd = open( fname, O_RDONLY );
	if ( -1 == fd ) goto cleanup;

	n = read( fd, &msg, sizeof(message) );
	if ( n != sizeof(message) ) {

		rc = n < 0 ? CSPI_E_SYSTEM : CSPI_E_DSCPROTO;
	}
	else {

		*msg_val = msg.val;
		rc = msg.status;
                /* Interpret DSC errors */
                if (0 > rc) {
                    errno = -rc;
                    rc = CSPI_E_SYSTEM;
                }
	}
	close(fd);

cleanup:
	unlink( fname );
	return rc;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Convert gain in dBm to attenuator values.
 *  Returns sum of the first and second channel attenuators or -1 on error.
 *
 *  @param gain Gain value to convert [dBm].
 */
int ebpp_toattn( const int gain )
{
	FILE *fp = fopen( "/opt/dsc/gain.conf", "r" );
	if ( !fp ) return -1;

	int rc = -1;
	char *line = 0;
	size_t size = 0;

	ssize_t nread;
	while ((nread = getline( &line, &size, fp )) != -1 ) {

		const char *p = line;
		while ( isspace(*p) ) { ++p; }
		if (*p == '\0' || *p == '#') continue;

		int g, a1, a2;
		sscanf( p, "%d %d %d", &g, &a1, &a2 );
		if ( gain == g ) {
			rc = a1 + a2;
			break;
		}
	}
	if ( line ) free( line );
	fclose(fp);
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
 *  @param p Pointer to a size_t variable with decimation to validate.
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
 *  @param p Pointer to a size_t variable with switch value to validate.
 */
inline int ebpp_is_validswitch( const void *p )
{
	const size_t switches = *(size_t *)p;
	return switches == CSPI_SWITCH_AUTO || 
               switches >= CSPI_SWITCH_MIN  ||
               switches <= CSPI_SWITCH_MAX;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates gain setting.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a int variable with gain value to validate.
 */
inline int ebpp_is_validgain( const void *p )
{
	const int MIN = -80, MAX = 0;

	const int gain = *(int *)p;
	return gain >= MIN && gain <= MAX;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates ACG setting.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a size_t variable with AGC value to validate.
 */
inline int ebpp_is_validagc( const void *p )
{
	const size_t agc = *(size_t *)p;
	return CSPI_AGC_AUTO == agc || CSPI_AGC_MANUAL == agc;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates DSC setting.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a size_t variable with DSC value to validate.
 */
inline int ebpp_is_validdsc( const void *p )
{
	const size_t dsc = *(size_t *)p;
	return dsc == CSPI_DSC_OFF    ||
	       dsc == CSPI_DSC_UNITY  ||
               dsc == CSPI_DSC_AUTO   ||
	       dsc == CSPI_DSC_SAVE_LASTGOOD;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates interlock parameters.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a size_t variable with mode to validate.
 */
inline int ebpp_is_validilk( const void *p )
{
	const size_t mode = *(size_t *)p;
	return mode == CSPI_ILK_DISABLE ||
	       mode == CSPI_ILK_ENABLE  ||
	       mode == CSPI_ILK_ENABLE_GAINDEP;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Adapts interlock overflow limit to the low-level interface.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a int variable with overflow limit to adapt.
 */
inline int ebpp_set_overflowlimit( const void *p )
{
        const unsigned int MAX = 2047;
	unsigned int *limit = (unsigned int *)p;	// cast away const
        unsigned int adc_count = *limit;

        if ( MAX < adc_count ) return 0;

        // NOTE: limit = sqr(acd_count)/256
        adc_count *= adc_count;
        *limit = ( *limit << 16 ) | ( ( adc_count >> 8) & 0x0000ffff );
	return 1;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Adapts interlock overflow limit to the CSPI interface.
 *  Returns 1.
 *  @param p Pointer to a int variable with overflow limit to adapt.
 */
inline int ebpp_get_overflowlimit( const void *p )
{
	unsigned int *limit = (unsigned int *)p;	// cast away const
        *limit >>= 16;
	return 1;
}

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates interlock overflow duration parameter.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a int variable with overflow duration to adapt.
 */
inline int ebpp_is_validoverflowdur( const void *p )
{
        /* Checks made in Libera driver due to DDC decimation dependency. */
        return 1;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Validates interlock gain limit to the low-level interface.
 *  On success, returns 1. Otherwise, returns 0.
 *  @param p Pointer to a int variable with gain limit to adapt.
 */
inline int ebpp_set_gainlimit( const void *p )
{
	int *limit = (int *)p;	// cast away const
	const int attn = ebpp_toattn( *limit );
	if ( -1 == attn ) return 0;
		
	*limit = ( ((unsigned int) *limit) << 16 ) | ( attn & 0x0000ffff );
	return 1;
}

//--------------------------------------------------------------------------

/** Private. EBPP specific. Local to this module only.
 *
 *  Adapts interlock gain limit to the CSPI interface.
 *  Returns 1.
 *  @param p Pointer to a int variable with gain limit to adapt.
 */
inline int ebpp_get_gainlimit( const void *p )
{
	int *limit = (int *)p;	// cast away const
	*limit >>= 16;
	return 1;
}

//--------------------------------------------------------------------------

/** Private.
 *  Represents DSC parameter traits.
 */
typedef struct tagDSC_param_traits {
	const size_t mask;		//!< Parameter bitmask.
	const size_t msg_type;	//!< DSC server request type.
	VALIDATOR validate;		//!< Validation function.
}
DSC_param_traits;

/** Private.
 *  Maps a DSC parameter to the corresponding DSC_param_traits.
 */
typedef	struct tagDSC_param_map {
	const void *valaddr;		//!< Pointer to parameter value.
	DSC_param_traits traits;	//!< Parameter traits.
}
DSC_param_map;

/** Private.
 *  Macro to create a write-only DSC_param_map entry.
 */
#define DSC_WPARAM( NAME, VALADDR, FNC ) \
	{VALADDR, {CSPI_ENV_ ## NAME, DSCD_SET_ ## NAME, FNC}}

/** Private.
 *  Macro to create a read-only DSC_param_map entry.
 */
#define DSC_RPARAM( NAME, VALADDR, FNC ) \
	{VALADDR, {CSPI_ENV_ ## NAME, DSCD_GET_ ## NAME, FNC}}

//--------------------------------------------------------------------------

int ebpp_dsc_handle_params( DSC_param_map *p, CSPI_BITMASK flags )
{
	int rc = CSPI_OK;

	// Assume DSC_param_map vector is null terminated!
	for ( ; p->valaddr; ++p ) {

		const DSC_param_traits *traits = &p->traits;
		if ( flags & traits->mask ) {

			if ( traits->validate && !traits->validate( p->valaddr ) ) {

				return CSPI_E_INVALID_PARAM;
			}

			rc = ebpp_dsc_message( traits->msg_type, (int *)p->valaddr );
			if ( CSPI_OK != rc ) break;
		}
	}
	return rc;
}

//--------------------------------------------------------------------------

int ebpp_setdscparam(Environment *e, const CSPI_ENVPARAMS *p,
                     CSPI_BITMASK flags)
{
	if ( !(fcntl(e->fd, F_GETFL) & O_RDWR) ) {

		errno = EPERM;
		return CSPI_E_SYSTEM;
	}

	const DSC_param_map map[] = {

		DSC_WPARAM( SWITCH, &p->switches, ebpp_is_validswitch ),
		DSC_WPARAM( AGC,    &p->agc,      ebpp_is_validagc    ),
		DSC_WPARAM( GAIN,   &p->gain,     ebpp_is_validgain   ),
		DSC_WPARAM( DSC,    &p->dsc,      ebpp_is_validdsc    ),
		{0, {0,0}}
	};

	return ebpp_dsc_handle_params( (DSC_param_map *)map, flags );
}

//--------------------------------------------------------------------------

int ebpp_getdscparam(Environment *e, const CSPI_ENVPARAMS *p,
                     CSPI_BITMASK flags)
{
	DSC_param_map map[] = {

		DSC_RPARAM( SWITCH, &p->switches, 0 ),
		DSC_RPARAM( GAIN,   &p->gain,     0 ),
		DSC_RPARAM( AGC,    &p->agc,      0 ),
		DSC_RPARAM( DSC,    &p->dsc,      0 ),
		{0, {0,0}}
	};

	return ebpp_dsc_handle_params( map, flags );
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

#define CSPI_ENV_ILK_MODE			CSPI_ENV_ILK
#define CSPI_ENV_ILK_XLOW			CSPI_ENV_ILK
#define CSPI_ENV_ILK_XHIGH			CSPI_ENV_ILK
#define CSPI_ENV_ILK_YLOW			CSPI_ENV_ILK
#define CSPI_ENV_ILK_YHIGH			CSPI_ENV_ILK
#define CSPI_ENV_ILK_OVERFLOW_LIMIT	CSPI_ENV_ILK
#define CSPI_ENV_ILK_OVERFLOW_DUR	CSPI_ENV_ILK
#define CSPI_ENV_ILK_GAIN_LIMIT		CSPI_ENV_ILK

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

		PARAM( ILK_MODE, &(p->ilk).mode,  ebpp_is_validilk ),

		PARAM( ILK_XLOW,  &(p->ilk).Xlow,  0 ),
		PARAM( ILK_XHIGH, &(p->ilk).Xhigh, 0 ),
		PARAM( ILK_YLOW,  &(p->ilk).Ylow,  0 ),
		PARAM( ILK_YHIGH, &(p->ilk).Yhigh, 0 ),

		PARAM( ILK_OVERFLOW_DUR,   &(p->ilk).overflow_dur,  ebpp_is_validoverflowdur ),
		PARAM( ILK_OVERFLOW_LIMIT, &(p->ilk).overflow_limit, ebpp_set_overflowlimit ),

		PARAM( ILK_GAIN_LIMIT, &(p->ilk).gain_limit, ebpp_set_gainlimit ),

		// Note: must be null terminated!
		{ 0, {0, 0} },
	};

	// Call "base" method to handle common params.
	int rc = base_setenvparam( e, p, flags );
	if ( CSPI_OK == rc ) {

		// Handle ebpp-specific params.
		rc = handle_params( e->fd, (Param_map *)map, flags, SET );
		if ( CSPI_OK == rc ) {

			// Handle DSC and DSC-related params in a special way.
			rc = ebpp_setdscparam( e, p, flags );
		}
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

		PARAM( ILK_MODE, &(p->ilk).mode,  0 ),

		PARAM( ILK_XLOW,  &(p->ilk).Xlow,  0 ),
		PARAM( ILK_XHIGH, &(p->ilk).Xhigh, 0 ),
		PARAM( ILK_YLOW,  &(p->ilk).Ylow,  0 ),
		PARAM( ILK_YHIGH, &(p->ilk).Yhigh, 0 ),

		PARAM( ILK_OVERFLOW_DUR,   &(p->ilk).overflow_dur,   0 ),
		PARAM( ILK_OVERFLOW_LIMIT, &(p->ilk).overflow_limit, ebpp_get_overflowlimit ),

		PARAM( ILK_GAIN_LIMIT, &(p->ilk).gain_limit, ebpp_get_gainlimit ),

		// Note: must be null terminated!
		{ 0, {0, 0} },
	};

	if ( !flags ) return CSPI_OK;

	// Call "base" method to handle common params.
	int rc = base_getenvparam( e, p, flags );
	if ( CSPI_OK == rc ) {

		// Handle ebpp-specific params.
		rc = handle_params( e->fd, map, flags, GET );
		if ( CSPI_OK == rc ) {

			// Handle DSC and DSC-related params in a special way.
			rc = ebpp_getdscparam( e, p, flags );
		}
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

	// Handle DD connection params, specific to EBPP.
	if ( flags & CSPI_CON_DEC ) {

		if ( -1 == con->fd ) return CSPI_E_SEQUENCE;	// Connect first!
		if ( CSPI_MODE_DD != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_EBPP *q = (CSPI_CONPARAMS_EBPP *)p;
		if ( !ebpp_is_validdec( &q->dec ) ) return CSPI_E_INVALID_PARAM;

		ASSERT(-1 != con->fd);
		if (-1 == ioctl( con->fd, LIBERA_IOC_SET_DEC, &q->dec )) return CSPI_E_SYSTEM;
	}

        // Handle SA connection params, specific to EBPP.
	if ( flags & CSPI_CON_SANONBLOCK ) {

		if ( -1 == con->fd ) return CSPI_E_SEQUENCE;	// Connect first!
		if ( CSPI_MODE_SA != con->mode ) return CSPI_E_ILLEGAL_CALL;

		const CSPI_CONPARAMS_EBPP *q = (CSPI_CONPARAMS_EBPP *)p;

		ASSERT(-1 != con->fd);
                int sa_flags = fcntl( con->fd, F_GETFL, 0);
                if (-1 == sa_flags) return CSPI_E_SYSTEM;
                if ( q->nonblock )
                    sa_flags |= O_NONBLOCK;
                else
                    sa_flags &= ~O_NONBLOCK;
		if (-1 == fcntl( con->fd, F_SETFL, sa_flags )) return CSPI_E_SYSTEM;
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

		const CSPI_CONPARAMS_EBPP *q = (CSPI_CONPARAMS_EBPP *)p;

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
int ebpp_transform_dd( const void *in, void *out )
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

/** Private. EBPP specific. Local to this module only.
 *
 *  Transforms ADC atom's values from [0..4095] to [-2047..2048].
 *  @param in Not used.
 *  @param out Pointer to the CSPI_ADC_ATOM to transform.
 */
inline int ebpp_transform_adc( const void *in, void *out )
{
	ASSERT(in == (const void*)out);
	CSPI_ADC_ATOM *p = out;

	if (p->chA > 2048) p->chA -= 4096;
	if (p->chB > 2048) p->chB -= 4096;
	if (p->chC > 2048) p->chC -= 4096;
	if (p->chD > 2048) p->chD -= 4096;

	return 0;
}

//--------------------------------------------------------------------------

CSPI_AUX_FNC custom_getdefaultop( const Connection *p )
{
	ASSERT(p);
	const int mode = p->mode;

	if (CSPI_MODE_DD==mode || CSPI_MODE_PM==mode) {
		return ebpp_transform_dd;
	}
	if (CSPI_MODE_ADC==mode) {
		return ebpp_transform_adc;
	}
	return 0;
}
