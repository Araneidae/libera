/* $Id: libera_pll.h,v 1.13 2006/09/26 10:52:37 ales Exp $ */

//! \file libera_pll.h
//! Declares interface for Libera PLL daemons.

#if !defined(_LIBERA_PLL_H)
#define _LIBERA_PLL_H

// TODO: #include <sys/types.h>	// defines pid_t

#ifdef __cplusplus
extern "C" {
#endif

/** LMTD process identifier (PID) pathname. */
#define LMTD_PID_PATHNAME           "/var/run/lmtd.pid"
/* Command FIFO for receiving LMTD commands. */
#define LMTD_COMMAND_FIFO           "/tmp/lmtd.command"
/* Status FIFO for reporting LMTD status. */
#define LMTD_STATUS_FIFO            "/tmp/lmtd.status"


/** LSTD process identifier (PID) pathname. */
#define LSTD_PID_PATHNAME           "/var/run/lstd.pid"

/** Libera event device. */
#define LIBERA_EVENT_FIFO_PATHNAME  "/dev/libera.event"

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


/* LMTD defines */
#define LMTD_DEFAULT_MCPRESC     84663UL
#define LMTD_DEFAULT_DEC           129UL
#define LMTD_DEFAULT_HARMONIC      416UL
#define LMTD_DEFAULT_UNOMINAL   0x6600

#define FS_FR     6
#define QS_FR     8
#define M_FR      7

#define TRUE  1
#define FALSE 0

/* LSTD defines */
#define LSTD_DEFAULT_UNOMINAL   0x54a4


#define ERR_LMT_UNLOCK          30000
#define ERR_LST_UNLOCK          30000
  

//--------------------------------------------------------------------------
// Interface.


#ifdef __cplusplus
}
#endif
#endif	// _LIBERA_PLL_H
