/* $Id: libera_pll.h,v 1.13 2006/09/26 10:52:37 ales Exp $ */

/*
LIBERA PLL DAEMONS - Libera GNU/Linux PLL daemons
Copyright (C) 2004-2006 Instrumentation Technologies
Copyright (C) 2006-2007 Michael Abbott, Diamond Light Source Ltd.

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

//! \file libera_pll.h
//! Declares interface for Libera PLL daemons.

#if !defined(_LIBERA_PLL_H)
#define _LIBERA_PLL_H

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


/* LMTD internal state, as reports on the status pipe. */
typedef enum
{
    LMTD_NO_CLOCK,          // Clock lost
    LMTD_FREQUENCY_SEEK,    // Seeking requested frequency
    LMTD_PHASE_SEEK,        // Wide band phase lock
    LMTD_PHASE_LOCKED,      // Narrow band phase lock

    LMTD_LOCK_STATE_COUNT = LMTD_PHASE_LOCKED+1
} LMTD_LOCK_STATE;



#ifdef __cplusplus
}
#endif
#endif	// _LIBERA_PLL_H
