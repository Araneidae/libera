// $Id: cspi_impl.h,v 1.10 2006/01/06 22:01:06 miha Exp $

//! \file cspi_impl.h
//! Private (implementation specific) CSPI structs and functions.

#if !defined(_CSPI_IMPL_H)
#define _CSPI_IMPL_H

#include <signal.h>
#include <pthread.h>

#include "debug.h"

#if !defined(_CSPI_H)
#error ERROR: Include cspi.h first!
#endif	// _CSPI_H

#ifdef __cplusplus
extern "C" {
#endif

/** Typedef representing a byte. */
typedef unsigned char byte;

//--------------------------------------------------------------------------
// Notification section.

/** Private.
 *  Signal handler type definition.
 *  Assumption: the signal and accompanying information is sent
 *  using the sigqueue function.
 */
typedef void (*SIGACTION)(int, siginfo_t *, void *);

/** Private.
 *  One and only signal handler.
 *  Handles LIBERA_SIGNAL and dispatches event to all
 *  connections with a registered event handler.
 *
 *  @param signum  See sigaction man page or glibc documentation.
 *  @param si      See sigaction man page or glibc documentation.
 *  @param notused See sigaction man page or glibc documentation.
 */
void signal_handler( int signum, siginfo_t *si, void *notused );

/** Private.
 *  Allows member-specific code to hook into the signal handler.
 *
 *  @param p Pointer to CSPI_EVENTHDR structure.
 */
void signal_handler_hook( const CSPI_EVENTHDR *p );

/** Private.
 *  Resets LIBERA_SIGNAL action to system default action.
 *  See glibc sigaction for information on return values.
 */
int reset_sighandler();

//--------------------------------------------------------------------------
// Internal environment and connection related section.

/** Private. Represents global CSPI library parameters. */
typedef struct tagLibrary {
	/** CSPI version the application confirms to (R/W). */
	int version;
	/** Super user flag: 0 or 1 (R/W). */
	int superuser;
} Library;

/** Private. Magic numbers. */
typedef enum {
	MAGIC_ENV = 230799,	//!< Environment magic.
	MAGIC_CON = 230271,	//!< Connection magic.
} MAGIC;

/** Typedef. See struct tagConnection for more information. */
typedef struct tagConnection Connection;	// forward decl

/** Private.
 *  Environment represents a global context in which to retrieve data.
 *  With the environment is associated data that is global in nature or
 *  common to all connections on that environment.
 *
 *  Changes to the environment are implemented in a thread-safe manner.
 *  Thus, each connection on the environment can run in its own thread.
 */
typedef struct tagEnvironment {
	MAGIC type_id;				//!< Magic number.
	size_t usage_count;			//!< Usage (reference) count.
	pthread_mutex_t mutex;		//!< Protects from concurrent modifications.
	SIGACTION sigaction;		//!< Signal handler.
	size_t connection_count;	//!< Number of connections on the environment.
	Connection *head;			//!< Connection list.
	int fd;						//!< Configuration device file descriptor.
	int trig_mode;				//!< Trigger mode.
	Library module;				//!< Global CSPI module parameters.
} Environment;

/** Private.
 *  Declares one and only environment instance. Shared by all connections.
 */
extern Environment environment;

/** Private.
 *  Connection structure represents a connection to the specific Libera
 *  data source, for instance Data On Demand (DD).
 *  Assumption: a connection is not shared between threads!
 */
struct tagConnection {
	MAGIC type_id;				//!< Magic number.
	int connection_id;			//!< Unique connection id.
	int mode;					//!< Mode of operation (see CSPI_MODE).
	int fd;						//!< Device file descriptor.
	int pid;					//!< Pid to register with the event daemon.
	CSPI_BITMASK event_mask;	//!< Event mask to register with the evet daemon.
	CSPI_EVENTHANDLER handler; 	//!< Notification message handler.
	void *user_data;			//!< User data passed to handler on each call.
	CSPI_TIMESTAMP timestamp;	//!< Time stamp of the last DD read.
	Environment *environment;	//!< Environment that owns the connection.
	Connection *next;			//!< Next object in the connection list.
	Connection *prev;			//!< Previous object in the connection list.
};

//--------------------------------------------------------------------------
// Validation section.

/** Private.
 *  Validates environment handle.
 *  Returns 1 if h points to an environment structure, 0 otherwise.
 *  @param h Handle to test.
 */
static inline int is_henv( CSPIHANDLE h )
{
	ASSERT( h == &environment );
	return h && ( MAGIC_ENV == ((Environment*) h)->type_id );
}

/** Private.
 *  Validates connection handle.
 *  Returns 1 if h points to a connection structure, 0 otherwise.
 *  @param h Handle to test.
 */
static inline int is_hcon( CSPIHANDLE h )
{
	return h && ( MAGIC_CON == ((Connection*) h)->type_id );
}

/** Private.
 *  Returns 1 if mode represents a streaming data source,
 *  for instance Slow Acq. (SA).
 *  @param mode Mode to test.
 */
static inline int is_streamingmode( const int mode )
{
	return CSPI_MODE_SA  == mode;
}

//--------------------------------------------------------------------------
// Validation section -- cont'd:
// Used by address and thus not declared inline.

/** Private.
 *  Validates mode of operation.
 *  Returns 1 on success, or 0 if p represents invalid or unknown
 *  (CSPI_MODE_UNKNOWN) mode of operation.
 *  @param p Pointer to mode to validate.
 */
int is_validmode( const void *p );

/** Private.
 *  Validates trigger mode.
 *  Returns 1 on success, or 0 if p represents invalid or unknown
 *  (CSPI_TRIGMODE_UNKNOWN) mode of operation.
 *  @param p Pointer to trigger mode to validate.
 */
int is_validtrigmode( const void *p );

/** Private.
 *  Validates CSPI version.
 *  Returns 1 on success, or 0 otherwise.
 *  @param p Pointer to version number to validate.
 */
int is_validversion( const void *p );

//--------------------------------------------------------------------------
// Internal environment and connection related section.

/** Private.
 *  Allocates environment handle.
 *  Returns CSPI_OK on success, or one of the values returned
 *  by custom_initenv otherwise.
 *  @param p Pointer to environment handle to allocate.
 */
int alloc_env(CSPIHANDLE *p);

/** Private.
*   Deallocates environment handle.
 *  @param h Environment handle to free.
 */
void free_env( CSPIHANDLE h );

/** Private.
 *  Initializes base-part of an allocated environment structure.
 *  Returns CSPI_OK on success, or CSPI_E_SYSTEM or value returned
 *  by the test_drvmismatch,
 *
 *  @param h Environment handle to initialize.
 */
int base_initenv( CSPIHANDLE h );

/** Private.
 *  Asserts driver version.
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_SYSTEM
 *  CSPI_E_VERSION.
 */
int test_drvmismatch(int fd);

/** Private.
 *  Initializes custom (derived) part of an allocated environment
 *  structure. Overriden by each member of the Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param h Environment handle to initialize.
 */
int custom_initenv( CSPIHANDLE h );

/** Private.
 *  Proceeds in the reverse order as init_env.
 *  Returns CSPI_OK on success or one of the following errors:
 *  CSPI_E_SYSTEM,
 *  CSPI_E_SEQUENCE.
 *
 *  @param h Environment handle to destroy.
 */
int destroy_env( CSPIHANDLE h );

/** Private.
 *  Allocates and initializes a new connection structure.
 *  On success, p is set to point to a newly allocated connection,
 *  or to 0 otherwise.
 *  Returns CSPI_OK on success, or one of the values returned by
 *  custom_initcon otherwise.
 *
 *  @param h Environment handle.
 *  @param p Connection handle to allocate.
 */
int alloc_con( CSPIHANDLE h, CSPIHANDLE *p );

/** Private.
 *  Deallocates (frees) a memory block occupied by the connection.
 *  @param h Connection handle to free.
 */
void free_con( CSPIHANDLE h );

/** Private.
 *  Initializes base connection members to suitable defaults.
 *  Assigns connection p to the environment h.
 *  Returns CSPI_OK.
 *
 *  @param h Environment handle.
 *  @param p Connection handle to initialize.
 */
int base_initcon( CSPIHANDLE h, CSPIHANDLE p );

/** Private.
 *  Called by alloc_con to initialize custom (derived) connection
 *  members to suitable defaults. Overriden by each member of the
 *  Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param h Environment handle.
 *  @param p Connection handle to initialize.
 */
int custom_initcon( CSPIHANDLE h, CSPIHANDLE p );

/** Private.
 *  Proceeds in the reverse order as base_initcon.
 *  Returns CSPI_OK on success, or CSPI_E_SYSTEM otherwise.
 *
 *  @param h Connection handle to destroy.
 */
int destroy_con( CSPIHANDLE h );

/** Private.
 *  Inserts connection into the environment's connection list.
 *  @param h Connection handle to insert.
 */
void insert_con( CSPIHANDLE h );

/** Private.
 *  Removes connection from the environment's connection list.
 *  @param h Connection handle to remove.
 */
void remove_con( CSPIHANDLE h );

/** Private.
 *  Control Libera events.
 *  Issues a request to the Libera event daemon to start or stop
 *  receiving events.
 *  Returns 0 on success, or -1  otherwise (the errno is set in this case).
 *
 *  @param pid  Process id (pid) to register with the event daemon.
 *  @param mask Event mask. Disables events if 0.
 */
int event_ctl( int pid, size_t mask );

/** Private.
 *  Called by cspi_setlibparam to handle CSPI parameters
 *  in a thread safe way.
 *
 *  @param module Pointer to Library structure.
 *  @param p      Pointer to CSPI_LIBPARAMS structure with values to set.
 *  @param flags  Bitmask specifying which values to set.
 */
int set_libparam( Library *module, const CSPI_LIBPARAMS *p, size_t flags );

/** Private.
 *  Called by cspi_getlibparam to handle CSPI parameters
 *  in a thread safe way.
 *
 *  @param module Pointer to Library structure.
 *  @param p      Pointer to CSPI_LIBPARAMS structure to fill.
 *  @param flags  Bitmask specifying which values to fill.
 */
int get_libparam( Library *module, CSPI_LIBPARAMS *p, size_t flags );

//--------------------------------------------------------------------------

/** Private.
 *  Represents a validation function to assert the values of
 *  environment and connection parameters.
 *  The function should return non-zero (true) if p is pointing 
 *  to a valid content, or 0 otherwise.
 *
 *  @param p Pointer to value to vallidate.
 */
typedef int (*const VALIDATOR)( const void *p );

/** Private.
 *  Represents parameter traits.
 */
typedef struct tagParam_traits {
	const size_t mask;		//!< Parameter bit mask.
	const int code;			//!< Request code.
	VALIDATOR validate;		//!< Validation function.
}
Param_traits;

/** Private.
 *  Maps a library or environment parameter to
 *  the corresponding Param_traits.
 */
typedef	struct tagParam_map {
	const void *field;		//!< Pointer to parameter value.
	Param_traits traits;	//!< Parameter traits.
}
Param_map;

/** Private.
 *  Macro to create a Param_traits entry.
 */
#define PARAM_TRAITS( NAME, FNC ) \
        {CSPI_ENV_ ## NAME, LIBERA_CFG_ ## NAME, FNC}

/** Private.
 *  Macro to create a Param_map entry.
 */
#define PARAM( NAME, FIELD, FNC ) {FIELD, PARAM_TRAITS(NAME,FNC)}

//--------------------------------------------------------------------------

/** Private.
 *  Called to assign values to the base-part of the Environment structure.
 *  Returns CSPI_OK on success, or one of the errors returned by set_param.
 *  
 *  @param e     Pointer to environment structure.
 *  @param p     Pointer to CSPI_ENVPARAMS structure with values to set.
 *  @param flags Bitmask specifying which values to set.
 */
int base_setenvparam( Environment *e, const CSPI_ENVPARAMS *p, CSPI_BITMASK flags );

/** Private.
 *  Called to retrieve values from the base-part of the Environment
 *  structure. Returns CSPI_OK on success, or one of the errors
 *  returned by handle_params.
 *  
 *  @param e     Pointer to environment.
 *  @param p     Pointer to CSPI_ENVPARAMS structure to fill.
 *  @param flags Bitmask specifying which values to fill.
 */
int base_getenvparam( Environment *e, CSPI_ENVPARAMS *p, CSPI_BITMASK flags );

/** Private.
 *  Called by cspi_setenvparam to assign values to the custom (derived)
 *  part of the environment structure. Overriden by each member of the
 *  Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param e     Pointer to environment.
 *  @param p     Pointer to CSPI_ENVPARAMS structure with values to set.
 *  @param flags Bitmask specifying which values to set.
 */
int custom_setenvparam( Environment *e, const CSPI_ENVPARAMS *p,
                        CSPI_BITMASK flags );

/** Private.
 *  Called by cspi_getenvparam to retrieve values from the custom (derived)
 *  part of the environment structure. Overriden by each member of the
 *  Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param e     Pointer to environment.
 *  @param p     Pointer to CSPI_ENVPARAMS structure to fill.
 *  @param flags Bitmask specifying which values to fill.
 */
int custom_getenvparam( Environment *e, CSPI_ENVPARAMS *p, CSPI_BITMASK flags );

enum { GET=0, SET };

/** Private.
 *  Handle parameter input/output.
 *  Returns CSPI_OK on success, or one of the errors returned by set_param
 *  or get_param.
 *
 *  @param fd    Device file descriptor.
 *  @param p     Pointer to null-terminated Param_map vector.
 *  @param flags Bitmask specifying which values to get/set.
 *  @param op    Operation type (GET or SET).
 */
int handle_params( int fd, Param_map *p, CSPI_BITMASK flags, int op );

/** Private.
 *  Set the value of a single parameter.
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  @param fd     Device file descriptor.
 *  @param p      Pointer to parameter value.
 *  @param traits Pointer to parameter traits.
 */
int set_param( int fd, int *p, const Param_traits *traits );

/** Private.
 *  Get the value of a single parameter.
 *  Returns CSPI_OK on success, or CSPI_E_SYSTEM otherwise.
 *
 *  @param fd     Device file descriptor.
 *  @param p      Pointer to parameter value.
 *  @param traits Pointer to parameter traits.
 */
int get_param( int fd, int *p, const Param_traits *traits );

/** Private.
 *  Returns device filename, based on mode of operation.
 *  @param mode Mode of operation.
 */
const char* get_devicename(int mode);

/** Private.
 *  Called by cspi_setconparam to assign values to the custom (derived)
 *  part of the connection structure. Overriden by each member of the
 *  Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param con   Pointer to connection.
 *  @param p     Pointer to CSPI_CONPARAMS structure with values to set.
 *  @param flags Bitmask specifying which values to set.
 */
int custom_setconparam( Connection *con, const CSPI_CONPARAMS *p,
                        CSPI_BITMASK flags );

/** Private.
 *  Called to assign values to the base-part of the Connection structure.
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *  
 *  @param con   Pointer to connection structure.
 *  @param p     Pointer to CSPI_CONPARAMS structure with values to set.
 *  @param flags Bitmask specifying which values to set.
 */
int base_setconparam( Connection *con, const CSPI_CONPARAMS *p, CSPI_BITMASK flags );

/** Private.
 *  Called by cspi_getconparam to retrieve values from the custom (derived)
 *  part of the connection structure. Overriden by each member of the
 *  Libera family.
 *  Returns CSPI_OK on success, or an implementation-specific error.
 *
 *  @param con   Pointer to environment.
 *  @param p     Pointer to CSPI_CONPARAMS structure to fill.
 *  @param flags Bitmask specifying which values to fill.
 */
int custom_getconparam( Connection *con, CSPI_CONPARAMS *p, CSPI_BITMASK flags );

/** Private.
 *  Called to retrive values of the base-part of the Connection structure.
 *  Returns CSPI_OK.
 *  
 *  @param con   Pointer to connection structure.
 *  @param p     Pointer to CSPI_CONPARAMS structure to fill.
 *  @param flags Bitmask specifying which values to fill.
 */
int base_getconparam( Connection *con, CSPI_CONPARAMS *p, CSPI_BITMASK flags );

//--------------------------------------------------------------------------
// Data transformation and conversion routines.

/** Private.
 *  Returns default aux. DD operator for given connection.
 *  Overriden by each member of the Libera family.
 *
 *  @param p Connection pointer.
 */
CSPI_AUX_FNC custom_getdefaultop( const Connection *p );

int custom_initop();

/** Private.
 *  Read from DD (or PM) data source.
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_MALLOC,
 *  CSPI_E_SYSTEM,
 *  CSPI_W_INCOMPLETE (warning, not an error).
 *
 *  @param p     Connection pointer.
 *  @param dest  Pointer to destination buffer.
 *  @param count Number of samples to read.
 *  @param nread Number of samples actually read.
 *  @param op    Aux. operator to apply to each sample.
 */
int read_dd( Connection *p,
             void *dest, size_t count,
             size_t *nread,
             CSPI_AUX_FNC op );

/** Private.
 *  Read from ADC data source.
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_SYSTEM,
 *  CSPI_W_INCOMPLETE (warning, not an error).
 *
 *  @param p     Connection pointer.
 *  @param dest  Pointer to destination buffer.
 *  @param count Number of samples to read.
 *  @param nread Number of samples actually read.
 *  @param op    Aux. operator to apply to each sample.
 */
int read_adc( Connection *p,
              void *dest, size_t count,
              size_t *nread,
              CSPI_AUX_FNC op );

#ifdef __cplusplus
}
#endif
#endif	// _CSPI_IMPL_H
