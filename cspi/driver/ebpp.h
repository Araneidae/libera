/* $Id: ebpp.h,v 1.34 2007/11/08 16:46:11 tomazb Exp $ */

/** \file ebpp.h */
/** Public include file for Libera Electron Beam Position Processor (EBPP). */

/*
LIBERA - Libera GNU/Linux device driver
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

#ifndef _EBPP_H_
#define _EBPP_H_

/** Libera magic number */
#define LIBERA_MAGIC 0xeb00

/** EBPP specific CFG parameters. */
typedef enum {
    LIBERA_CFG_XOFFSET = LIBERA_CFG_CUSTOM_FIRST,        //!< Horizontal electrical/magnetic offset.
    LIBERA_CFG_YOFFSET,        //!< Vertical electrical/magnetic offset.
    LIBERA_CFG_QOFFSET,        //!< Electrical offset.
    LIBERA_CFG_KX,             //!< Horizontal calibration coefficient.
    LIBERA_CFG_KY,             //!< Vertical calibration coefficient.
    LIBERA_CFG_ILK_XLOW,           //!< Horizontal interlock threshold (LOW).
    LIBERA_CFG_ILK_XHIGH,          //!< Horizontal interlock threshold (HIGH).
    LIBERA_CFG_ILK_YLOW,           //!< Vertical interlock threshold (LOW).
    LIBERA_CFG_ILK_YHIGH,          //!< Vertical interlock threshold (HIGH).
    LIBERA_CFG_ILK_MODE,       //!< Interlock  mode
    LIBERA_CFG_ILK_OVERFLOW_LIMIT,     //!< Interlock overflow limit (ADC count)
    LIBERA_CFG_ILK_OVERFLOW_DUR,  //!< Interlock overflow duration (ADC clock periods)
    LIBERA_CFG_ILK_GAIN_LIMIT, //!< Gain limit (dBm) for gain-dependant interlock
    LIBERA_CFG_CUSTOM_LAST,
    // TODO: Check with CSPI regarding FIRST/LAST
    LIBERA_CFG_SW,
    LIBERA_CFG_LEVEL,
    LIBERA_CFG_AGC,
    LIBERA_CFG_DSC,
    LIBERA_CFG_ILKSTATUS,
    LIBERA_CFG_PMOFFSET,
    LIBERA_CFG_EXTSWITCH,
    LIBERA_CFG_SWDELAY,
    LIBERA_CFG_DFA,
    LIBERA_CFG_DEC_DDC,
    LIBERA_CFG_DDC_MAFLENGTH,
    LIBERA_CFG_DDC_MAFDELAY,
    LIBERA_CFG_TRIGDELAY,
    // Add new parameters before this line.
#define LIBERA_CFG_COEFF   256 // position of coefficients in the param array
#define LIBERA_CFG_NOTCH_MAX 5 // hardcoded number of coefficients
    LIBERA_CFG_NOTCH1 = LIBERA_CFG_COEFF,  // start of FIR coefficients
    LIBERA_CFG_NOTCH2 = LIBERA_CFG_NOTCH1 + LIBERA_CFG_NOTCH_MAX,
    LIBERA_CFG_POLYPHASE_FIR = LIBERA_CFG_NOTCH2 + LIBERA_CFG_NOTCH_MAX,
#define LIBERA_CFG_FIR_MAX 192 // max polyphase FIR = 3 * max decimation
} LIBERA_CFG_EBPP_GENERIC;

#define LIBERA_DD_CIRCBUF_ATOMS   2097152
/** No. of DD atoms in one DMA block */
#define LIBERA_DMA_BLOCK_ATOMS  128
/* DMA buffer size = PAGE_SIZE << LIBERA_DMA_PAGE_ORDER */
#define LIBERA_DMA_PAGE_ORDER     5
/** No. of DD atoms in DMA fifo  - Must be power of 2 and sync
 *  with LIBERA_DMA_PAGE_ORDER */
#define LIBERA_DMA_FIFO_ATOMS  4096
#define LIBERA_DMA_FIFO_MASK (LIBERA_DMA_FIFO_ATOMS - 1)
/* Libera EBPP Data on Demand (DD) atom */
/* NOTE: The size of libera_atom_dd_t structure is important. 
 *       The minimal lenght of libera_atom_dd_t is 4 bytes and it must be 
 *       integer (4-byte) aligned!
 */
typedef struct {
    libera_S32_t cosVa;
    libera_S32_t sinVa;
    libera_S32_t cosVb;
    libera_S32_t sinVb;
    libera_S32_t cosVc;
    libera_S32_t sinVc;
    libera_S32_t cosVd;
    libera_S32_t sinVd;
} libera_atom_dd_t;


/* Libera EBPP ADC-rate Data (ADC) atom */
/* NOTE: The size of libera_atom_adc_t structure is important. 
 *       The minimal lenght of libera_atom_adc_t is 4 bytes and it must be 
 *       integer (4-byte) aligned!
 */
typedef struct {
    short ChD;
    short ChC;
    short ChB;
    short ChA;
} libera_atom_adc_t;


/* Libera EBPP Slow Acquisition (SA) atom */
/* NOTE: The size of libera_atom_sa_t structure is important. 
 *       PAGE_SIZE MUST be a multiple of sizeof(libera_atom_sa_t) for proper 
 *       buffer wrapping. Pad this structure to the nearest common denominator 
 *       of PAGE_SIZE and sizeof(libera_atom_sa_t).
 *       The minimal lenght of libera_atom_sa_t is 4 bytes and it must be 
 *       integer (4-byte) aligned!
 */
typedef struct {
    /* 4 amplitudes */
    libera_S32_t Va, Vb, Vc, Vd;
    /* 4 synthetic values -> Sum, Q, X, Y */
    libera_S32_t Sum, Q, X, Y;
    /* Cx and Cy for FF */
    libera_S32_t Cx, Cy;
    /* 6 values reserved for future use */
    libera_S32_t reserved[6];
} libera_atom_sa_t;

#define SA_FIFO_DEPTH    32 /* Top/botoom half SA buffer length in atoms. */
#define SA_SIZE_WIDTH     1
#define SA_LOG           64 // SA terminal pipe buffer depth. Must be 2^n.


#endif // _EBPP_H_
