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

/* Definitions shared between lmtd.c and lstd.c but not exported to other
 * clients. */

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
  
