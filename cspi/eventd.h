// $Id: eventd.h,v 1.5 2005/10/24 11:15:11 miha Exp $

//! \file eventd.h
//! Declares interface for CSPI Event Daemon.

#if !defined(_EVENTD_H)
#define _EVENTD_H

#include <sys/types.h>	// defines pid_t

#ifdef __cplusplus
extern "C" {
#endif

/** Event signal. */
#define LIBERA_SIGNAL SIGUSR1

/** Process identifier (PID) pathname. */
#define EVENTD_PID_PATHNAME		"/var/run/leventd.pid"

/** Request FIFO (named pipe) pathname. */
#define EVENTD_REQ_FIFO_PATHNAME	"/tmp/leventd.fifo"

/** Libera event device. */
#define LIBERA_EVENT_FIFO_PATHNAME	"/dev/libera.event"

//--------------------------------------------------------------------------
// Interface.

/** Represents event daemon request. */
typedef struct {
	pid_t pid;
	size_t mask;
}
Request;

/** Typedef. See struct tagListener for more information. */
typedef struct tagListener Listener;	// forward decl

/** Represents a member of the listener list. */
struct tagListener {
	pid_t pid;
	size_t mask;
	Listener *prev, *next;
};

#ifdef __cplusplus
}
#endif
#endif	// _EVENTD_H
