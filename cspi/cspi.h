// $Id: cspi.h,v 1.38 2006/11/21 12:22:17 ales Exp $

//! \file cspi.h
//! Public CSPI header file.

#if !defined(_CSPI_H)
#define _CSPI_H

#include <sys/types.h>	// size_t definition
#include <unistd.h>		// SEEK_* definitions

#include "libera.h"		// public driver interface

#if !defined(CSPI_VER)
/** CSPI version. */
#define CSPI_VER 0x0010
#endif

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------------
// Error handling.

/** Warning codes returned by the CSPI functions.
 *  See warn_list[] for a list of coresponding descriptions.
 */
enum {
	/** Success (not a warning). */
	CSPI_OK = 0,

	/** Partially completed request. */
	CSPI_W_INCOMPLETE,
};

/** Error codes returned by the CSPI functions.
 *  See err_list[] for a list of coresponding descriptions.
 *
 *  Note: CSPI_E_SYSTEM error is returned when a system-level call
 *  inside a CSPI function fails. Additional information about the
 *  error can be obtained by inspecting the errno value immediately
 *  after the call that returned CSPI_E_SYSTEM.
 */
enum {
	/** Unknown error. */
	CSPI_E_UNKNOWN = -1,

	/** Invalid handle. */
	CSPI_E_INVALID_HANDLE = -2,

	/** Invalid function call sequence. */
	CSPI_E_SEQUENCE = -3,

	/** Invalid function parameter. */
	CSPI_E_INVALID_PARAM = -4,

	/** System-level call failed. */
	CSPI_E_SYSTEM = -5,

	/** Invalid mode of operation or mode not set. */
	CSPI_E_INVALID_MODE = -6,

	/** Illegal CSPI call. */
	CSPI_E_ILLEGAL_CALL = -7,

	/** Failed to allocate memory. */
	CSPI_E_MALLOC = -8,

	/** Driver version mismatch. */
	CSPI_E_VERSION = -9,

	/** DSC server protocol error. */
	CSPI_E_DSCPROTO = -10,
};

/** Maximum length of the error description string. */
#define CSPI_MAX_MSG_LEN 256

/** \brief Returns string describing error code.
 *
 *  The function returns the appropriate error description string,
 *  or the unknown error message if the error code is unknown.
 *  \param errnum Error code.
 */
const char* cspi_strerror(int errnum);

//--------------------------------------------------------------------------
// Async. event section.

/** Libera asynchronous events. */
typedef enum {
	/** User defined event. */
	CSPI_EVENT_USER			= LIBERA_EVENT_USER,

	/** Low-level fifo overflow. */
	CSPI_EVENT_OVERFLOW		= LIBERA_EVENT_OVERFLOW,

	/** Configuration change. */
	CSPI_EVENT_CFG			= LIBERA_EVENT_CFG,

	/** Slow Acq. (SA) sample available. */
	CSPI_EVENT_SA			= LIBERA_EVENT_SA,

	/** Interlock fired. */
	CSPI_EVENT_INTERLOCK	= LIBERA_EVENT_INTERLOCK,

	/** Post Mortem trigger. */
	CSPI_EVENT_PM			= LIBERA_EVENT_PM,

	/** Fast Application trigger. */
	CSPI_EVENT_FA			= LIBERA_EVENT_FA,

	/** GET Trigger trigger. */
	CSPI_EVENT_TRIGGET		= LIBERA_EVENT_TRIGGET,

	/** SET Trigger trigger. */
	CSPI_EVENT_TRIGSET		= LIBERA_EVENT_TRIGSET,
}
CSPI_EVENTMASK;

/** Event specific values for CSPI_EVENT_OVERFLOW. */
enum {
	/** Data on Demand (DD) fifo overflow in FPGA. */
	CSPI_OVERFLOW_DD_FPGA = LIBERA_OVERFLOW_DD_FPGA,

	/** Slow Acq. (SA) fifo overflow in FPGA. */
	CSPI_OVERFLOW_SA_FPGA = LIBERA_OVERFLOW_SA_FPGA,

	/** Slow Acq. (SA) fifo overflow in driver. */
	CSPI_OVERFLOW_SA_DRV  = LIBERA_OVERFLOW_SA_DRV,
};

/** Event specific values for event CSPI_EVENT_FA. */
enum {
	/** Fast Acq. (FA) MC trigger #0. */
	CSPI_TRIG_FA_MC0 = LIBERA_TRIG_FA_MC0,

	/** Fast Acq. (FA) MC trigger #1. */
	CSPI_TRIG_FA_MC1 = LIBERA_TRIG_FA_MC1,

	/** Fast Acq. (FA) SC trigger #0. */
	CSPI_TRIG_FA_SC0 = LIBERA_TRIG_FA_SC0,

	/** Fast. Acq. (FA) SC trigger #1. */
	CSPI_TRIG_FA_SC1 = LIBERA_TRIG_FA_SC1,
};

/** Groups all async. notification events. */
#define CSPI_NOTICEMASK		(CSPI_EVENT_SA|CSPI_EVENT_INTERLOCK)

/** Groups all async. error events. */
#define CSPI_ERRORMASK		(CSPI_EVENT_OVERFLOW)

/** Groups all async. trigger events. */
#define CSPI_TRIGGERMASK	(CSPI_EVENT_PM|CSPI_EVENT_FA|CSPI_EVENT_TRIGGET|\
	                         CSPI_EVENT_TRIGSET)

/** Represents async. event header.
 *  CSPI_EVENTHDR is simply a synonim for the type employed in the Libera
 *  driver to represent async. event. 
 *
 *  This structure is used to pass information about an asynchronous event
 *  signaled by the driver, such as event id and event specific value or
 *  code. CSPI provides this information as part of the larger CSPI_EVENT
 *  structure.
 *
 *  See libera.h for more information on member variables and a list of
 *  possible values.
 */
typedef libera_event_t CSPI_EVENTHDR;

/** Represents async. event.
 *  This structure expands the CSPI_EVENTHDR with a pointer to arbitrary
 *  data to pass to the event handler each time it is called. This parameter
 *  is not interpreted in any way by the CSPI.
 */
typedef struct {
	/** Event header. */
	CSPI_EVENTHDR hdr;
	/** Pointer to user data set with the cspi_setconparam. */
	void *user_data;
} CSPI_EVENT;

/** Type definition for event handler.
 *  To inform application about an async. event in the underlying
 *  layers, an event handler for each connection is called.
 *
 *  To dispatch event to the next connection in row, the handler
 *  should return a non-zero value. To stop dispatching the event,
 *  the handler should return 0.
 *
 *  @param msg Pointer to event structure.
 */
typedef int (*CSPI_EVENTHANDLER)( CSPI_EVENT *msg );

//--------------------------------------------------------------------------
// Handle related section.

/** Generic handle type. */
typedef void *		CSPIHANDLE;

/** Environment handle type.
 *  Environment handle represents a global context for all Libera
 *  operations. An environment handle is implemented in a thread-safe manner.
 *  Each connection on the environment can thus run in its own thread.
 */
typedef CSPIHANDLE	CSPIHENV;

/** Connection handle type.
 *  Connection handle represents a connection to a particular data
 *  source in the Libera. A connection handle is not thread safe and should
 *  not be shared among threads.
 */
typedef CSPIHANDLE	CSPIHCON;

//! Handle type identifiers.
enum {
	CSPI_HANDLE_ENV = 1,	//!< Environment handle.
	CSPI_HANDLE_CON,		//!< Connection handle.
};

/** Used to represent a bitmask. */
typedef size_t CSPI_BITMASK;

/** \brief Allocate an environment or connection handle.
 *
 *  This function is a generic function for allocating handles.
 *  Returns CSPI_OK on success or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  \param type Type of handle to allocate. Must be one of the
 *              following values: CSPI_HANDLE_ENV or CSPI_HANDLE_CON.
 *  \param h    Input handle as a context for the new handle to
 *              allocate. Set to 0 if 'type' is CSPI_HANDLE_ENV.
 *              Must be an environment handle, if 'type' is CSPI_HANDLE_CON.
 *  \param p    Pointer to a buffer in which to return the handle to the
 *              newly allocated data structure.
 */
int cspi_allochandle( int type, CSPIHANDLE h, CSPIHANDLE* p );

/** \brief Free an environment or connection handle.
 *
 *  This function is a generic function for freeing handles. It frees the
 *  resources associated with a specific environment or connection handle.
 *
 *  Prior to calling cspi_freehandle with a handle type of CSPI_HANDLE_ENV,
 *  an application must free all connections allocated on the environment.
 *  Otherwise, the function returns CSPI_E_SEQUENCE and the environment and
 *  any active connection remain valid.
 *
 *  Prior to calling cspi_freehandle with a handle type of CSPI_HANDLE_CON,
 *  an application must disconnect from data source with cspi_disconnect (if
 *  there is a connection on this handle). Otherwise, the function returns
 *  SQL_E_SEQUENCE and the connection remains valid.
 *
 *  Returns CSPI_OK on success or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SEQUENCE,
 *  CSPI_E_SYSTEM.
 *
 *  \param type Type of handle to free. Must be one of the following
 *              values: CSPI_HANDLE_ENV or CSPI_HANDLE_CON.
 *  \param h    The handle to be freed.
 */
int cspi_freehandle( int type, CSPIHANDLE h );

//--------------------------------------------------------------------------
// Library related section.

/** CSPI library parameters or attributes. */
typedef struct {
	/** CSPI version the application confirms to (R/W). NOT USED! */
	int version;
	/** Superuser flag: 0 (the default) or 1 (R/W). */
	int superuser;
}
CSPI_LIBPARAMS;

/** Helper macro for bitmasks. */
#define BIT(n)	(1 << (n))

/** Bitmasks corresponding to the CSPI_LIBPARAMS structure.
 *  See also CSPI_LIBPARAMS structure for descriptions.
 */
typedef enum {
	CSPI_LIB_VERSION	= BIT(0),	//!< CSPI version flag.
	CSPI_LIB_SUPERUSER	= BIT(1),	//!< CSPI superuser flag.
}
CSPI_LIBFLAGS;

/** \brief Set the CSPI module parameters.
 *
 *  For a list of parameters, see CSPI_LIBPARAMS. To request exclusive
 *  write access to Libera environment and common (global) parameters,
 *  the superuser field of the CSPI_LIBPARAMS structure should be set
 *  to 1.
 *
 *  The superuser value is stored but does not step in effect until
 *  an environment handle (a global context for Libera operations) is
 *  actually allocated.
 *
 *  The version must be set to CSPI_VER.
 *
 *  All successfully set parameters persist until the environment handle
 *  is released.
 *
 *  Returns CSPI_OK on success or CSPI_E_INVALID_PARAM if an error occurred.
 *
 *  @param p      Pointer to the CSPI_LIBPARAMS structure with values
 *                to set.
 *  @param flags  Bitmask specifying which parameters to set. This can be
 *                any combination of the CSPI_LIBFLAGS, for instance
 *                CSPI_LIB_VERSION | CSPI_LIB_SUPERSUER.
 */
int cspi_setlibparam( const CSPI_LIBPARAMS *p, CSPI_BITMASK flags );

/** \brief Retrieve the CSPI module parameters.
 *
 *  For a list of parameters, see CSPI_LIBPARAMS. Note that parameter
 *  values returned are those that are stored by the CSPI library and
 *  may not be in effect. See cspi_setlibparam for more information.
 *
 *  Returns CSPI_OK.
 *
 *  @param p     Pointer to the CSPI_LIBPARAMS structure in which to return
 *               results.
 *  @param flags Bitmask specifying which parameters to retrieve. This can
 *               be any combination of the CSPI_LIBFLAGS, for instance
 *               CSPI_LIB_VERSION | CSPI_LIB_SUPERSUER.
 */
int cspi_getlibparam( CSPI_LIBPARAMS *p, CSPI_BITMASK flags );

//--------------------------------------------------------------------------
// Configuration section.

/** Available TRIGGER trigger modes. */
typedef enum {
	/** Trigger mode not set. */
	CSPI_TRIGMODE_UNKNOWN	= LIBERA_TRIGMODE_UNKNOWN,
	/** GET Trigger trigger. */
	CSPI_TRIGMODE_GET		= LIBERA_TRIGMODE_GET,
	/** 'SET Trigger trigger. */
	CSPI_TRIGMODE_SET		= LIBERA_TRIGMODE_SET,
}
CSPI_TRIGMODE;

// fwd decl -- reqr'd to declare pub. CSPI func. cspi_set/getenvparam
struct tagCSPI_ENVPARAMS;

/** Represents environment parameters.
 *  The actual tagCSPI_ENVPARAMS structure is Libera family member specific
 *  and defined in a corresponding member-specific header.
 */
typedef struct tagCSPI_ENVPARAMS CSPI_ENVPARAMS;

/** Represents CSPI health environment parameters.
 *  Temperature, fans & PS voltages.
 */
typedef struct {
        int temp;
        int fan[2];
        int voltage[8];
}
cspi_health_t;

/** Represents CSPI PLL status environment parameters. */
typedef struct {
    unsigned long sc;
    unsigned long mc;
}
cspi_pll_t;

/** Declares environment parameters common to all members of the Libera
 *  family. Derived structures use this macro to declare 'base' members.
 */
#define CSPI_ENVPARAMS_BASE \
	/** Health info. Temperature, fans & PS voltages. */ \
	cspi_health_t health; \
	/** PLL status. */ \
	cspi_pll_t pll; \
	/** Trigger mode. See enum CSPI_TRIGGER (R/W). */ \
	int trig_mode;

/** Bitmasks corresponding to the common environment parameters.
 *  See CSPI_ENVPARAMS structure and CSPI_ENVPARAMS_BASE macro for more 
 *  information.
 */
typedef enum {
	CSPI_ENV_HEALTH 	= BIT(0),
	CSPI_ENV_TRIGMODE	= BIT(1),
	CSPI_ENV_PLL    	= BIT(2),
	// CSPI_ENV_reserved = BIT(2)...BIT(7),
}
CSPI_ENVFLAGS;

/** Helper macro to define custom environment bitmasks. */
#define CUSTOM_ENV_BIT(n)	BIT(8 + n)


/** \brief Set environment parameters.
 *
 *  For a list of parameters, see CSPI_ENVPARAMS. Note that parameters such
 *  as X and Y interlock are read-only and their value cannot be set.
 *  All successfully set environment parameters persist until the
 *  cspi_freehandle is called on the environment.
 *
 *  Returns CSPI_OK on success or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  \param h     Environment handle.
 *  \param p     Pointer to CSPI_ENVPARAMS structure with values to set.
 *  \param flags Bitmask specifying which parameters to set. This can be any
 *               combination of the CSPI_ENVFLAGS and Libera family member
 *               specific environment flags, for instance
 *               CSPI_ENV_TRIGMODE | CSPI_ENV_KX | CSPI_ENV_KY.
 */
int cspi_setenvparam( CSPIHENV h, const CSPI_ENVPARAMS *p, CSPI_BITMASK flags );

/** \brief Retrieve current environment settings.
 *
 *  For a list of parameters, see CSPI_ENVPARAMS. Cspi_getenvparam can be
 *  called at any time between the time an environment handle is allocated
 *  and freed. All successfully set environment parameters persist until
 *  cspi_freehandle is called on the environment.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  \param h     Environment handle.
 *  \param p     Pointer to CSPI_ENVPARAMS structure in which to return
 *               results.
 *  \param flags Bitmask specifying which parameters to retrieve. This can
 *               be any combination of the CSPI_ENVFLAGS and Libera family
 *               member specific environment flags, for instance
 *               CSPI_ENV_TRIGMODE | CSPI_ENV_XINTERLOCK.
 */
int cspi_getenvparam( CSPIHENV h, CSPI_ENVPARAMS *p, CSPI_BITMASK flags );

/** \brief Set Fast Application (FA) parameters.
 *
 *  Writes 'count' elements of data, each 'size' bytes long, to the 
 *  configuration block of the FA interface. Note that data is not
 *  interpreted in any way by the CSPI. Cspi_setenvparams_fa merely
 *  provides a configuration channel for the fast application embedded
 *  in the FPGA.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *  
 *  Size of element must be aligned on a 4-byte boundary!
 *
 * @param h      Environment handle.
 * @param offset Offset from the beginning of the FA config. block.
 * @param pbuf   Pointer to data to write..
 * @param size   Size of a data element.
 * @param count  Number of data elements.
 */
int cspi_setenvparam_fa( CSPIHENV h, size_t offset,
                         const void *pbuf, size_t size, size_t count );

/** \brief Retrieve current Fast Application (FA) parameters.
 *
 *  Reads 'count' elements of data, each 'size' bytes long, from the
 *  configuration block of the FA interface.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  Size of element must be aligned on a 4-byte boundary!
 *
 * @param h      Environment handle.
 * @param offset Offset from the beginning of the FA config. block.
 * @param pbuf   Pointer to destination to retrieve the data to.
 * @param size   Size of a data element.
 * @param count  Number of data elements.
 */
int cspi_getenvparam_fa( CSPIHENV h, size_t offset,
                         void *pbuf, size_t size, size_t count );

/** Available Libera modes of operation.
 *  Note: only a subset may apply to a Libera family member.
 */
typedef enum {
	/** Mode of operation not set. */
	CSPI_MODE_UNKNOWN = LIBERA_MODE_UNKNOWN,

	/** Data-on-Demand (DD) mode. */
	CSPI_MODE_DD = LIBERA_MODE_DD,

	/** Slow Acquisition (SA) mode. */
	CSPI_MODE_SA = LIBERA_MODE_SA,

	/** Post-Mortem (PM) mode. */
	CSPI_MODE_PM = LIBERA_MODE_PM,

	/** ADC-rate data (ADCRD) mode. */
	CSPI_MODE_ADC = LIBERA_MODE_ADC,
}
CSPI_MODE;

/** Declares connection parameters common to all members of the Libera
 *  family. Derived structures use this macro to declare 'base' members.
 */
#define CSPI_CONPARAMS_BASE \
	/** Mode of operation. See CSPI_MODE for a list of available modes. */ \
	int mode; \
	/** Event handler. */ \
	CSPI_EVENTHANDLER handler; \
	/** User data passed to the handler on each call. */ \
	void *user_data; \
	/** Event mask. */ \
	CSPI_BITMASK event_mask

/** Common connection parameters or attibutes.
 *  Derived, connection-specific structures add additional members.
 */
typedef struct {
	CSPI_CONPARAMS_BASE;
}
CSPI_CONPARAMS;

/** Bitmasks corresponding to the CSPI_CONPARAMS structure.
 *  See CSPI_CONPARAMS structure and CSPI_CONPARAMS_BASE macro for
 *  descriptions.
 */
typedef enum {
	CSPI_CON_MODE		= BIT(0),
	CSPI_CON_HANDLER	= BIT(1),
	CSPI_CON_USERDATA	= BIT(2),
	CSPI_CON_EVENTMASK	= BIT(3),
/*	CSPI_CON_reserved	= BIT(4) - BIT(7), */
}
CSPI_CONFLAGS;

/** Helper macro to define custom connection bitmasks. */
#define CUSTOM_CON_BIT(n)	BIT(8 + n)

/** \brief Set connection parameters.
 *
 *  For a list of parameters, see CSPI_CONPARAMS and derived structures.
 *
 *  CSPI_CONPARAMS structure represents parameters common to all types
 *  of connections. Depending on the mode of operation and Libera family
 *  member, additional connection parameters may be available.
 *  In this case, an application should call cspi_setconparam with a pointer
 *  to the appropriate CSPI_CONPARAMS-derived structure instead.
 *
 *  See the Libera family member specific header file for information on
 *  available connection parameters.
 *
 *  All CSPI_CONPARAMS-derived structures begin with CSPI_CONPARAMS and
 *  then add connection specific parameters.
 *
 *  An application can call cspi_setconparam at any time between the time
 *  a connection is opened and closed with cspi_connect and cspi_disconnect,
 *  respectively. All successfully set CSPI_CONPARAMS parameters persist until
 *  cspi_freehandle is called on the connection. All other successfully set
 *  connection parameters related to the undelying hardware persist even after
 *  an application terminates.
 *
 *  Note: event mask can be set at any time, but will only be activated when
 *        the event handler is set!
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_E_SEQUENCE,
 *  CSPI_E_SYSTEM.
 *
 *  \param h     Connection handle.
 *  \param p     Pointer to the CSPI_CONPARAMS or CSPI_CONPARAMS-derived
 *               structure with values to set.
 *  \param flags Bitmask specifying which parameters to set. This can be any
 *               combination of CSPI_CONFLAGS and flags for the
 *               CSPI_CONPARAMS-derived structure.
 *               Example: CSPI_CON_MODE | CSPI_CON_HANDLER.
 *
 */
int cspi_setconparam( CSPIHCON h, const CSPI_CONPARAMS *p, CSPI_BITMASK flags );

/** \brief Retrieve connection parameters.
 *
 *  See cspi_setconparam for more information.
 *
 *  Returns CSPI_OK on success, CSPI_W_REPEAT on partial success,
 *  or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_E_SEQUENCE,
 *  CSPI_E_SYSTEM.
 *
 *  \param h     Connection handle.
 *  \param p     Pointer to the CSPI_CONPARAMS or CSPI_CONPARAMS-derived
 *               structure to fill in.
 *  \param flags Bitmask specifying which parameters to retrieve. This can
 *               be any combination of CSPI_CONFLAGS and flags for the
 *               CSPI_CONPARAMS-derived structure.
 *               CSPI_CONFLAGS_FF if p points to CSPI_CONPARAMS_FF,
 *               CSPI_CONFLAGS_BN if p points to CSPI_CONPARAMS_BN.
 *               Example: CSPI_CON_MODE | CSPI_CON_HANDLER.
 */
int cspi_getconparam( CSPIHCON h, CSPI_CONPARAMS *p, CSPI_BITMASK flags );

//--------------------------------------------------------------------------
// Data retrieval section.

/** \brief Connect to a Libera data source.
 *
 *  This function opens a Libera device file for reading and associates
 *  it with given connection handle.
 *  Note: device file to open is determined based on mode of operation.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_MODE,
 *  CSPI_E_SEQUENCE,
 *  CSPI_E_SYSTEM.
 *
 * \param h  Connection handle.
 */
int cspi_connect( CSPIHCON h );

/** \brief Disconnect from a Libera data source.
 *
 *  This function disconnects from and closes the device file associated
 *  with given connection handle.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_SYSTEM.
 *
 * \param h Connection handle.
 */
int cspi_disconnect( CSPIHCON h );

/** Defines available time formats for cspi_seek. */
typedef enum {
	CSPI_SEEK_MT = SEEK_CUR,	//!< Machine time.
	CSPI_SEEK_ST = SEEK_SET,	//!< System time.
	CSPI_SEEK_TR = SEEK_END,	//!< Trigger time.
} CSPI_SEEK;

/** \brief Reposition read offset.
 *
 *  This function sets current data retrieval point (offset) in the History
 *  Buffer for Data on Demand (DD) and Post Mortem (PM) modes of operation.
 *  Cannot be used with a streaming data source such as Slow Acq. (SA).
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_INVALID_PARAM,
 *  CSPI_E_SEQUENCE,
 *  CSPI_E_SYSTEM.
 *
 *  Note: for diagnostic purposes, if origin is CSPI_SEEK_ST and offset
 *  is set to 0, current system time is used instead.
 *
 *  @param h      Connection handle.
 *  @param offset Pointer to a 64-bit integer representing the data 
 *                retrieval point to seek. The function should be passed 
 *                a pointer to struct timespec or a pointer to 64-bit 
 *                integer value, depending on whether origin is CSPI_SEEK_ST 
 *                or CSPI_SEEK_MT, respectively.
 *  @param origin Time format used in specifying the data retrieval point.
 *                Must be of type CSPI_SEEK. If set to CSPI_SEEK_TR, the
 *                offset parameter is ignored and current retrieval point
 *                is set to the time of last trigger instead.
 */
int cspi_seek( CSPIHCON h, unsigned long long *offset, int origin );

/** Calls cspi_read_ex with a predefined, default auxiliary function to
 *  transform a Data On Demand (DD) sample from raw (CSPI_DD_RAWATOM) to
 *  synthetic (CSPI_DD_ATOM) format.
 *
 *  See cspi_read_ex and structures CSPI_DD_RAWATOM, CSPI_DD_ATOM for
 *  more information about parameters and return values.
 *  Example for EBPP:
 *
 *  Mode of operation | Default action
 *  ---------------------------------------------------------------
 *  DD, PM              convert I,Q vals to amplitudes and position
 *  SA, ADC             do nothing
 *
 *  @param h     Connection handle.
 *  @param dest  Pointer to destination buffer to receive data.
 *               The buffer must be large enough to accomodate the 
 *               requested number of samples.
 *  @param count Number of samples to copy.
 *  @param nread Pointer to size_t to recieve the number of samples
 *               actually copied.
 */
int cspi_read( CSPIHCON h, void *dest, size_t count, size_t *nread );

/** Represents auxiliary fuction to pass to the cspi_read_ex. */
typedef int (*CSPI_AUX_FNC)( const void *in, void *out );

/** \brief Read from Data on Demand (DD) device.
 *
 *  Attempts to read a specified number of samples from a non-
 *  streaming data device such as History Buffer (DD mode), PM Buffer
 *  (PM mode) or ADC-rate Data Buffer (ADC mode).
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_E_MALLOC,
 *  CSPI_E_SYSTEM,
 *  CSPI_E_SEQUENCE,
 *  CSPI_W_INCOMPLETE (warning, not an error).
 *
 *  Cspi_read_ex is an extension of the cspi_read function.
 *  An application can call cspi_read_ex any time between the time a
 *  connection is opened and closed with cspi_connect ans cspi_disconnect,
 *  respectively.
 *
 *  @param h     Connection handle.
 *  @param dest  Pointer to destination buffer to receive data.
 *               The buffer must be large enough to accomodate the 
 *               requested number of samples.
 *  @param count Number of samples to copy.
 *  @param nread Pointer to size_t to recieve the number of samples
 *               actually copied.
 *  @param op    Auxiliary function to apply to each sample before copying
 *               it to the destination buffer. Can be used to perform
 *               a custom transformation on each sample. The function
 *               should return 0 on success or a non-zero value to stop
 *               the processing. If 0 is passed instead of a function, no
 *               transformation takes place and raw samples are read into
 *               the destination buffer.
 *
 */
int cspi_read_ex( CSPIHCON h, void *dest, size_t count, size_t *nread,
                  CSPI_AUX_FNC op  );

/** Represents a timestamp -- a (System Time, Machine Time) pair. */
typedef libera_timestamp_t CSPI_TIMESTAMP;

/** Represents a high resolution (set)timestamp -- System Time, Machine Time + offset. */
typedef libera_HRtimestamp_t CSPI_SETTIMESTAMP;

/** \brief Retrieve a timestamp.
 *
 *  Retrieves a timestamp associated with the last successfull cspi_read
 *  or cspi_read_ex call.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_INVALID_PARAM,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_E_SEQUENCE.
 *
 *  @param h  Connection handle.
 *  @param ts Pointer to the CSPI_TIMESTAMP structure to receive the
 *            timestamp of the first sample in a vector of samples
 *            returned by the last successfull cspi_read or cspi_read_ex
 *            call.
 */
int cspi_gettimestamp( CSPIHCON h, CSPI_TIMESTAMP *ts );

/** \brief Read from Slow Acquisition (SA) device.
 *
 *  Attempts to read a single SA sample into a user specified buffer.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_E_ILLEGAL_CALL,
 *  CSPI_E_SEQUENCE,
 *  CSPI_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  An application can call cspi_get any time between the time a
 *  connection is opened and closed with cspi_connect and cspi_disconnect,
 *  respectively.
 *
 *  @param h     Connection handle.
 *  @param atom  Pointer to the destination buffer to fill in.
 */
int cspi_get( CSPIHCON h, void *atom );

//--------------------------------------------------------------------------
// Sync. event section.

/** Available time formats. */
typedef enum {
	CSPI_TIME_MT = BIT(0),	//!< Machine time.
	CSPI_TIME_ST = BIT(1),	//!< System time.
}
CSPI_TIMEFLAGS;

/** \brief Synchronize time on Libera.
 *
 *  Sets the machine time and/or system time on Libera at next 'SET'
 *  TRIGGER trigger.
 *
 *  Note: 'SET' TRIGGER trigger is a one-shot trigger. That is, after
 *        the TRIGGER trigger, the old trigger mode is restored again.
 *
 *  Returns CSPI_OK on success, or one of the following errors:
 *  CSPI_E_INVALID_HANDLE,
 *  CSPI_INVALID_PARAM,
 *  CSPI_E_SYSTEM.
 *
 *  @param h     Environment handle.
 *  @param ts    Pointer to CSPI_TIMESTAMP structure with time to set.
 *  @param flags Bitmask specifying which time to set. This can be
 *               any combination of CSPI_TIMEFLAGS flags.
 */
int cspi_settime( CSPIHENV h, CSPI_SETTIMESTAMP *ts, CSPI_BITMASK flags );

#if defined(EBPP)
#include "ebpp.h"		// EBPP-specific declarations
#elif defined(BBFP)
#include "bbfp.h"		// BBFP-specific declarations
#elif defined(HBPP)
#include "hbpp.h"		// HBPP-specific declarations
#else
#error Must define Libera family member!
#endif

#ifdef __cplusplus
}
#endif

#endif	// _CSPI_H
