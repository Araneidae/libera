/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 *
 * This file is derived from the files libera.h and ebpp.h, part of
 *
 *  LIBERA - Libera GNU/Linux device driver
 *  Copyright (C) 2004-2006 Instrumentation Technologies
 *
 * However this file has been substiantially modified to cope with
 * incremental and growing incompatibilities with the slowly changing Libera
 * driver interface, and now only includes those definitions directly used by
 * the EPICS driver. */


/* Unfortunately Instrumentation Technologies have been rather sloppy in the
 * 1.80 to 2.00 upgrade, and managed to badly break backwards compatibility.
 * Fortunately this was done on a architecture boundary, so we can detect this
 * on the presence of the __ARM_EABI__ flag.  Unfortunately they also changed
 * numbers on the 2.00 to 2.02 upgrade, inserting LIBERA_CFG_PMDEC into the
 * middle of a block of definitions, so we'll need to take this into account
 * as well! */
#ifdef __ARM_EABI__
#define __EBPP_H_2
#else
#define __EBPP_H_1
#endif

#include <linux/ioctl.h>
#include <time.h>
#include <stdint.h>


/* Libera 64-bit time storage type. Used for (L)MT & (L)ST */
typedef uint64_t libera_hw_time_t; 

/* Libera userland timing pair, MT & ST */
typedef struct 
{
    struct timespec  st;    // System Time
    libera_hw_time_t mt;    // Machine Time
} libera_timestamp_t;

/* Libera High resolution userland timing pair, MT + D & ST */
#pragma pack(4)
typedef struct 
{
    struct timespec  st;
    libera_hw_time_t mt;
    uint32_t phase;
} libera_HRtimestamp_t;
#pragma pack()

/** Libera event structure */
typedef struct 
{
    int32_t id;             // Event ID
    int32_t param;          // Event specific parameter
} libera_event_t;

typedef struct
{
    uint32_t idx;           // Configuration parameter
    uint32_t val;           // Value for parameter
} libera_cfg_request_t;




/** Event IDs to be used in libera_event_t.id */
enum {
    LIBERA_EVENT_INTERLOCK  = (1 << 3),   // Interlock fired
    LIBERA_EVENT_PM         = (1 << 4),   // Post Mortem trigger
    LIBERA_EVENT_TRIGGET    = (1 << 6),   // GET Trigger trigger
    LIBERA_EVENT_TRIGSET    = (1 << 7),   // SET Trigger trigger
};


/* Libera driver ioctl definitions. */
enum {
    /* /dev/libera.cfg */
    /* Read configuration setting. */
    LIBERA_IOC_GET_CFG = _IOWR('l', 0, libera_cfg_request_t),
    /* Write configuration setting. */
    LIBERA_IOC_SET_CFG = _IOW('l', 0, libera_cfg_request_t),

    /* /dev/libera.dd */
    /* Set decimation, 1 or 64. */
    LIBERA_IOC_SET_DEC = _IOW('l', 96, uint32_t),
    /* Read timestamp for current waveform. */
    LIBERA_IOC_GET_DD_TSTAMP = _IOR('l', 97, libera_timestamp_t),

    /* /dev/libera.pm */
    /* Read timestamp for current waveform. */
    LIBERA_IOC_GET_PM_TSTAMP = _IOR('l', 128, libera_timestamp_t),

    /* /dev/libera.event */

    /* Set machine clock frequency control DAC. */
    LIBERA_EVENT_SET_DAC_A = _IOW('e', 0, int32_t),
    /* Set system clock frequency control DAC. */
    LIBERA_EVENT_SET_DAC_B = _IOW('e', 1, int32_t),
    /* Enable system clock events. */
    LIBERA_EVENT_ENABLE_SC_TRIG = _IOW('e', 2, int32_t),
    /* Enable machine clock events. */
    LIBERA_EVENT_ENABLE_MC_TRIG = _IOW('e', 3, int32_t),
    /* Set system clock time (on next trigger). */
    LIBERA_EVENT_SET_ST = _IOW('e', 4, libera_HRtimestamp_t),
    /* Set machine clock time (on next trigger). */
    LIBERA_EVENT_SET_MT = _IOW('e', 5, libera_HRtimestamp_t),
    /* Read next raw system clock event. */    
    LIBERA_EVENT_GET_SC_TRIGGER_9 = _IOR('e', 9, libera_hw_time_t),
    /* Read next raw machine time event. */    
    LIBERA_EVENT_GET_MC_TRIGGER_10 = _IOR('e', 11, libera_hw_time_t),
    /* Notify machine clock parameters to driver. */
    LIBERA_EVENT_SET_FLMC = _IOW('e', 12, uint32_t),
    /* Configure mask of events to be reported. */
    LIBERA_EVENT_SET_MASK = _IOW('e', 16, uint32_t),
    /* Load postmortem buffer from DD buffer. */
    LIBERA_EVENT_ACQ_PM = _IOW('e', 18, uint32_t),
    /* Notify machine clock parameters to driver. */
    LIBERA_EVENT_SET_MCPHI = _IOW('e', 19, libera_hw_time_t),
    /* Notify system clock parameters to driver. */
    LIBERA_EVENT_SET_SCPHI = _IOW('e', 20, libera_hw_time_t),
    /* Set frequency of RF IF oscillator. */
    LIBERA_EVENT_SET_NCO = _IOW('e', 23, uint32_t),
    /* Notify machine clock parameters to driver. */
    LIBERA_EVENT_SET_MCPLL = _IOW('e', 24, uint32_t),
    /* Notify system clock parameters to driver. */
    LIBERA_EVENT_SET_SCPLL = _IOW('e', 25, uint32_t),
};



#define TRIGGER_BIT(x)     (1 << (x+22))


/* Codes for LIBERA_IOC_{GET,SET}_CFG ioctl. */
enum {
    LIBERA_CFG_TRIGMODE = 0,            // Trigger mode
    LIBERA_CFG_MCPLL = 1,               // MC PLL status
    LIBERA_CFG_SCPLL = 2,               // SC PLL status
    LIBERA_CFG_XOFFSET = 128,           // Horizontal electrical offset.
    LIBERA_CFG_YOFFSET = 129,           // Vertical electrical offset.
    LIBERA_CFG_KX = 131,                // Horizontal calibration factor
    LIBERA_CFG_KY = 132,                // Vertical calibration factor
    LIBERA_CFG_ILK_XLOW = 133,          // Horizontal interlock threshold
    LIBERA_CFG_ILK_XHIGH = 134,         // Horizontal interlock threshold
    LIBERA_CFG_ILK_YLOW = 135,          // Vertical interlock threshold
    LIBERA_CFG_ILK_YHIGH = 136,         // Vertical interlock threshold
    LIBERA_CFG_ILK_MODE = 137,          // Interlock  mode
    LIBERA_CFG_ILK_OVERFLOW_LIMIT = 138, // Interlock overflow limit
    LIBERA_CFG_ILK_OVERFLOW_DUR = 139,
    LIBERA_CFG_ILK_GAIN_LIMIT = 140,
#ifdef __EBPP_H_2
    LIBERA_CFG_SR_ENABLE = 148,
    LIBERA_CFG_SR_AVERAGING_STOP = 150,
    LIBERA_CFG_SR_AVERAGE_WINDOW = 151,
    LIBERA_CFG_SR_START = 152,
    LIBERA_CFG_SR_WINDOW = 153,
    LIBERA_CFG_PMOFFSET = 163,
#endif
#ifdef __EBBP_H_1
    LIBERA_CFG_PMOFFSET = 147,
#endif
};


/* Libera EBPP Data on Demand (DD) atom: IQ data. */
typedef struct {
    int32_t cosVa;
    int32_t sinVa;
    int32_t cosVb;
    int32_t sinVb;
    int32_t cosVc;
    int32_t sinVc;
    int32_t cosVd;
    int32_t sinVd;
} libera_atom_dd_t;


/* Libera EBPP ADC-rate Data (ADC) atom: raw 16 bit ADC values. */
typedef struct {
    int16_t ChD;
    int16_t ChC;
    int16_t ChB;
    int16_t ChA;
} libera_atom_adc_t;


/* Libera EBPP Slow Acquisition (SA) atom: processed positions. */
typedef struct {
    /* 4 amplitudes */
    int32_t Va, Vb, Vc, Vd;
    /* 4 synthetic values -> Sum, Q, X, Y */
    int32_t Sum, Q, X, Y;
    /* Cx and Cy for FF */
    int32_t Cx, Cy;
    /* 6 values reserved for future use */
    int32_t reserved[6];
} libera_atom_sa_t;
