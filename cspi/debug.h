// $Id: debug.h,v 1.2 2005/08/31 09:14:30 miha Exp $

//! \file debug.h
//! Debugging macros and declarations.

#if !defined(_DEBUG_H)
#define _DEBUG_H

#include <syslog.h>

#if defined( DEBUG )
#include <assert.h>
#endif  // DEBUG

//--------------------------------------------------------------------------
// Debugging.

#if !defined(DEBUG)
#define DEBUG 0
#endif

#if DEBUG	// defined(DEBUG) && DEBUG>0

#define ASSERT(f)		assert(f)
#define VERIFY(f)		ASSERT(f)
#define DEBUG_ONLY(f)	(f)

#else	// DEBUG

#define ASSERT(f)		((void)0)
#define VERIFY(f)		((void)(f))
#define DEBUG_ONLY(f)	((void)0)

#endif	// !DEBUG

#if DEBUG < 3
#define DEBUG_ONLY_3(f)	((void)0)
#else
#define DEBUG_ONLY_3(f)	(f)
#endif

#if DEBUG < 2
#define DEBUG_ONLY_2(f)	((void)0)
#else
#define DEBUG_ONLY_2(f)	(f)
#endif

#if DEBUG < 1
#define DEBUG_ONLY_1(f)	((void)0)
#else
#define DEBUG_ONLY_1(f)	(f)
#endif

#define DEBUG_ONLY_0(f)	(f)

// The `##' token paste operator has a special meaning when placed between
// a comma and a variable argument. If you write
// #define eprintf(format, ...) fprintf (stderr, format, ##__VA_ARGS__)
// and the variable argument is left out when the eprintf macro is used,
// then the comma before the `##' will be deleted.
// This does not happen if you pass an empty argument, nor does it happen
// if the token preceding `##' is anything other than a comma.

/** Send crtitical message to the system logger. */
#define _LOG_CRIT( format, ... ) \
        DEBUG_ONLY_0( syslog( LOG_ERR, format, ##__VA_ARGS__ ) )

/** Send error message to the system logger. */
#define _LOG_ERR( format, ... ) \
        DEBUG_ONLY_1( syslog( LOG_ERR, format, ##__VA_ARGS__ ) )

/** Send warning message to the system logger. */
#define _LOG_WARNING( format, ... ) \
        DEBUG_ONLY_1( syslog( LOG_WARNING, format, ##__VA_ARGS__ ) )

/** Send normal, but significant message to the system logger. */
#define _LOG_NOTICE( format, ... ) \
        DEBUG_ONLY_2( syslog( LOG_NOTICE, format, ##__VA_ARGS__ ) )

/** Send informational message to the system logger. */
#define _LOG_INFO( format, ... ) \
        DEBUG_ONLY_2( syslog( LOG_INFO, format, ##__VA_ARGS__ ) )

/** Send debug-level message to the system logger. */
#define _LOG_DEBUG( format, ... ) \
        DEBUG_ONLY_3( syslog( LOG_DEBUG, format, ##__VA_ARGS__ ) )

/** Dumps expression to STDERR.
 *  This macro is only available in DEBUG build.
 *  Takes a format string as used in the run-time function printf.
 */
#define TRACE( f ) DEBUG_ONLY( fprintf( stderr, f ) )

/** Same as TRACE, but takes a format string plus one argument
 *  (one variable that is dumped to STDERR).
 */
#define TRACE1( f, p ) DEBUG_ONLY( fprintf( stderr, f, p ) )

/** Same as TRACE, but takes a format string plus two arguments
 *  (two variables that are dumped to STDERR).
 */
#define TRACE2( f, p, q ) DEBUG_ONLY( fprintf( stderr, f, p, q ) )

/** Same as TRACE, but takes a format string plus three arguments
 *  (three variables that are dumped to STDERR).
 */
#define TRACE3( f, p, q, r ) DEBUG_ONLY( fprintf( stderr, f, p, q, r ) )

//--------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

//...

#ifdef __cplusplus
}
#endif
#endif	// DEBUG_H
