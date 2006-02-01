/* $Id: libera.h,v 1.1 2006/01/27 15:58:18 mga83 Exp $ */

/** \file libera.h */
/** Public include file for GNU/Linux Libera driver. */

/*
LIBERA - Libera GNU/Linux device driver
Copyright (C) 2004-2005 Instrumentation Technologies

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


#ifndef _LIBERA_H_
#define _LIBERA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>
#ifndef __KERNEL__
#  include <time.h>
#endif

#include <limits.h>

/** Libera magic number for ioctl() calls */
#define LIBERA_IOC_MAGIC  'l'
#define LIBERA_LOW_MAGIC  'h'


/** Libera 32-bit types */
typedef int libera_S32_t;
typedef unsigned int libera_U32_t;
/** Libera 64-bit */
typedef long long libera_S64_t;
typedef unsigned long long libera_U64_t;
/** Libera 64-bit time storage type. Used for MC & SC */
typedef unsigned long long libera_hw_time_t;
/** Libera timing pair, LMT & LST*/    
typedef struct 
{
    libera_hw_time_t lst;
    libera_hw_time_t lmt;
} libera_Ltimestamp_t;
/** Libera userland timing pair, MT & ST */    
typedef struct 
{
    struct timespec  st;
    libera_hw_time_t mt;
} libera_timestamp_t;


/* Libera event structure */
typedef struct 
{
    int msg_id;
    int msg_param;
} libera_event_t;
/* Notification messages to be used in libera_event_t.msg_id */
typedef enum
{
    LIBERA_NM_USER = UINT_MAX/2,
    LIBERA_NM_CFG = 0,
    LIBERA_NM_DD,
    LIBERA_NM_PM,
    LIBERA_NM_SA,
    LIBERA_NM_OVERFLOW,
    LIBERA_NM_TRIGGER,
} libera_nm_t;
/* Libera overflow types. Used to accompany LIBERA_NM_OVERFLOW msg_id*/
typedef enum
{
    LIBERA_OVERFLOW_DD_FPGA,
    LIBERA_OVERFLOW_SA_FPGA,
    LIBERA_OVERFLOW_SA_DRV,
} libera_overflow_t;
    

/* Libera Slow Acquisition (SA) atomic packet */
/* NOTE: The size of "libera_sa_atomic_t" structure is important. 
 *       PAGE_SIZE MUST be a multiple of sizeof(libera_sa_atomic_t) for proper 
 *       buffer wrapping. Pad this structure to the nearest common denominator 
 *       of PAGE_SIZE and sizeof(libera_sa_atomic_t).
 */
#define LIBERA_SA_ATOMIC_LEN 16
typedef struct
{
    /* 4 raw measurements */
    libera_S32_t Va, Vb, Vc, Vd;
    /* 4 synthetic values -> X, Z, Q & Sum */
    libera_S32_t X, Z, Q, Sum;
    /* Cx and Cz for FF */
    libera_S32_t Cx, Cz;
    /* 6 values reserved for future use */
    libera_S32_t reserved[6];
} libera_sa_atomic_t;


/* Libera Data on Demand (DD) atomic packet */
#define LIBERA_DD_ATOMIC_LEN 8
typedef struct
{
    libera_S32_t cosVa;
    libera_S32_t sinVa;
    libera_S32_t cosVb;
    libera_S32_t sinVb;
    libera_S32_t cosVc;
    libera_S32_t sinVc;
    libera_S32_t cosVd;
    libera_S32_t sinVd;
} libera_dd_atomic_t;


/** Available modes of operation. */
typedef enum {
	LIBERA_MODE_UNKNOWN = 0,	//!< Trigger mode unknown or not set.
	LIBERA_MODE_SA,			//!< Slow Acquisition mode.
	LIBERA_MODE_FF,			//!< Fast Feedback mode.
	LIBERA_MODE_PM,			//!< Post-Mortem mode.
	LIBERA_MODE_BN,			//!< Booster Normal mode.
	LIBERA_MODE_FT,			//!< First Turns mode.
	LIBERA_MODE_TT,			//!< Turn-by-Turn mode.
} LIBERA_MODE;


/* LIBERA ioctl() command identifiers */
/* NOTE: The ordinal numbers (2nd paramter to _IO* IOCTL macros) 
 *       are divided into subsets corresponding to each set/group of
 *       LIBERA commands. The upper bit (MSB) is group/subset bit and 
 *       the rest 7 bits are sequential number bits. This gives us room for
 *       hosting 128 commands per group.
 *       GET_ commands MUST use _IOR macro.
 *       SET_ commands MUST use _IOW macro.
 */
#define LIBERA_IOC_IS_SET_METHOD(number) ((number) & 0x01)
#define LIBERA_IOC_IS_GET_METHOD(number) (!((number) & 0x01))
#define LIBERA_IOC_MASK  0xC0 /* 2 MSB bits                       */
#define LIBERA_IOC_CFG      0 /* Common Configuration Parameters  */
#define LIBERA_IOC_SA      64 /* Slow Acquisition Parameters      */
#define LIBERA_IOC_FF     128 /* Fast Acquisition Parameters      */
#define LIBERA_IOC_DD     192 /* Data on Demand Parameters */


/* Libera CFG device parameter IOC tags */
typedef enum
{
    LIBERA_CFG_KX = LIBERA_IOC_CFG,
    LIBERA_CFG_KZ,
    LIBERA_CFG_XOFFSET,
    LIBERA_CFG_ZOFFSET,
    LIBERA_CFG_QOFFSET,
    LIBERA_CFG_MODE,
    LIBERA_CFG_XINTERLOCK,
    LIBERA_CFG_ZINTERLOCK,
    LIBERA_CFG_XLOW,
    LIBERA_CFG_XHIGH,
    LIBERA_CFG_ZLOW,
    LIBERA_CFG_ZHIGH,
    LIBERA_CFG_SERIAL,
    LIBERA_CFG_ATTN,
    LIBERA_CFG_SWITCH,
#ifdef DEBUG
    LIBERA_CFG_MODULERESET,
#endif
    LIBERA_CFG_NONE,  /* Needed for default */
} libera_cfg_tags_t;

/* Libera SA device parameter IOC tags */
typedef enum
{
    LIBERA_SA_NONE = LIBERA_IOC_SA,
} libera_sa_tags_t;

/* Libera FF device parameter IOC tags */
typedef enum
{
    LIBERA_FF_FFENABLE = LIBERA_IOC_FF,
} libera_ff_tags_t;

/* Libera DD device parameter IOC tags */
typedef enum
{
    LIBERA_DD_DEC = LIBERA_IOC_DD,
} libera_dd_tags_t;


enum libera_ioc_ids_t
{
    /***********************************/
    /* Common Configuration Parameters */
    /***********************************/

    /* Kx: horizontal calibration coefficient */
    LIBERA_IOC_GET_KX       = _IOR(LIBERA_IOC_MAGIC, 
				   LIBERA_CFG_KX, libera_S32_t),
    LIBERA_IOC_SET_KX       = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_KX, libera_S32_t),

    /* Kz: vertical calibration coefficient */
    LIBERA_IOC_GET_KZ       = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_KZ, libera_S32_t),
    LIBERA_IOC_SET_KZ       = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_KZ, libera_S32_t),

    /* Xoffset: electrical/magnetic horizontal offset */
    LIBERA_IOC_GET_XOFFSET  = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XOFFSET, libera_S32_t),
    LIBERA_IOC_SET_XOFFSET  = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XOFFSET, libera_S32_t),

    /* Zoffset: electrical/magnetic vertical offset */
    LIBERA_IOC_GET_ZOFFSET  = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZOFFSET, libera_S32_t),
    LIBERA_IOC_SET_ZOFFSET  = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZOFFSET, libera_S32_t),

    /* Qoffset: electrical offset */
    LIBERA_IOC_GET_QOFFSET  = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_QOFFSET, libera_S32_t),
    LIBERA_IOC_SET_QOFFSET  = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_QOFFSET, libera_S32_t),

    /* Mode: acquisition mode (1st turns, turn-by-turn, ...) */
    LIBERA_IOC_GET_MODE     = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_MODE, libera_U32_t),
    LIBERA_IOC_SET_MODE     = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_MODE, libera_U32_t),

    /* ILx: interlock in or out of safe range in horizontal plane */
    LIBERA_IOC_GET_XINTERLOCK = _IOR(LIBERA_IOC_MAGIC,
				     LIBERA_CFG_XINTERLOCK, libera_S32_t),

    /* ILz: interlock in or out of safe range in vertical plane */
    LIBERA_IOC_GET_ZINTERLOCK = _IOR(LIBERA_IOC_MAGIC,
				     LIBERA_CFG_ZINTERLOCK, libera_S32_t),

    /* Xlow: horizontal interlock threshold */
    LIBERA_IOC_GET_XLOW     = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XLOW, libera_S32_t),
    LIBERA_IOC_SET_XLOW     = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XLOW, libera_S32_t),

    /* Xhigh: horizontal interlock threshold */
    LIBERA_IOC_GET_XHIGH    = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XHIGH, libera_S32_t),
    LIBERA_IOC_SET_XHIGH    = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_XHIGH, libera_S32_t),

    /* Zlow: vertical interlock threshold */
    LIBERA_IOC_GET_ZLOW     = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZLOW, libera_S32_t),
    LIBERA_IOC_SET_ZLOW     = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZLOW, libera_S32_t),

    /* Zhigh: vertical interlock threshold */
    LIBERA_IOC_GET_ZHIGH    = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZHIGH, libera_S32_t),
    LIBERA_IOC_SET_ZHIGH    = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ZHIGH, libera_S32_t),

    /* Snum: Serial number */
    LIBERA_IOC_GET_SERIAL   = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_SERIAL, libera_S32_t),

    /* ATTN: Attenuators */
    LIBERA_IOC_GET_ATTN     = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ATTN, unsigned char[8]),
    LIBERA_IOC_SET_ATTN     = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_ATTN, unsigned char[8]),

    /* SWITCH: Switches */
    LIBERA_IOC_GET_SWITCH   = _IOR(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_SWITCH, libera_U32_t),
    LIBERA_IOC_SET_SWITCH   = _IOW(LIBERA_IOC_MAGIC,
				   LIBERA_CFG_SWITCH, libera_U32_t),


    /*******************************/
    /* Slow Acquisition Parameters */
    /*******************************/
    /* NONE */

    /*******************************/
    /* Fast Acquisition Parameters */
    /*******************************/
    LIBERA_IOC_GET_FFENABLE  = _IOR(LIBERA_IOC_MAGIC,
				    LIBERA_FF_FFENABLE, libera_S32_t),
    LIBERA_IOC_SET_FFENABLE  = _IOW(LIBERA_IOC_MAGIC,
				    LIBERA_FF_FFENABLE, libera_S32_t),


    /*****************************/
    /* Data on Demand Parameters */
    /*****************************/
    /* DEC: Decimation */
    LIBERA_IOC_GET_DEC       = _IOR(LIBERA_IOC_MAGIC, 
				    LIBERA_DD_DEC, libera_U32_t),
    LIBERA_IOC_SET_DEC       = _IOW(LIBERA_IOC_MAGIC,
				    LIBERA_DD_DEC, libera_U32_t),

    /* All DEBUG IDs have to be declared at the bottom of enum! */
#ifdef DEBUG
    LIBERA_IOC_MODULERESET  = _IOW(LIBERA_IOC_MAGIC, 
				   LIBERA_CFG_MODULERESET, libera_S32_t),
#endif

};


/**********************************/
/* Low Level (Housekeeping) IOCTL */
/**********************************/
typedef enum
{
    LIBERA_LOW_DAC_A,
    LIBERA_LOW_DAC_B,
    LIBERA_LOW_SC_TRIG,
    LIBERA_LOW_MC_TRIG,
    LIBERA_LOW_SC_TIME,
    LIBERA_LOW_MC_TIME,
    LIBERA_LOW_SC_EVENT,
    LIBERA_LOW_SC_TRIGGER_19,
    LIBERA_LOW_SC_TRIGGER_10,
    LIBERA_LOW_SC_TRIGGER_9,
    LIBERA_LOW_MC_TRIGGER_19,
    LIBERA_LOW_MC_TRIGGER_10,
    LIBERA_LOW_FLMC,
    LIBERA_LOW_CTIME,
    LIBERA_LOW_TRIG_TRIGGER,
    LIBERA_LOW_TRIGGER_BLOCK,
    LIBERA_LOW_EVENT,
    LIBERA_LOW_EVENTFLUSH,
    /* All DEBUG IDs have to be declared at the bottom of enum! */
#ifdef DEBUG
    LIBERA_LOW_PEEK_POKE,
    LIBERA_LOW_EVENTSIM,
#endif
} libera_low_tags_t;

#ifdef DEBUG
typedef struct 
{
    unsigned long offset;
    unsigned long value;
} libera_peek_poke_t;
#endif

enum libera_low_ids_t
{    
    LIBERA_LOW_SET_DAC_A       = _IOW(LIBERA_LOW_MAGIC,
				   LIBERA_LOW_DAC_A, libera_S32_t),
    LIBERA_LOW_SET_DAC_B       = _IOW(LIBERA_LOW_MAGIC,
				   LIBERA_LOW_DAC_B, libera_S32_t),
    LIBERA_LOW_SET_SC_EVENT    = _IOW(LIBERA_LOW_MAGIC,
				   LIBERA_LOW_SC_EVENT, libera_S32_t),
    LIBERA_LOW_ENABLE_SC_TRIG  = _IOW(LIBERA_LOW_MAGIC,
				   LIBERA_LOW_SC_TRIG, libera_S32_t),
    LIBERA_LOW_ENABLE_MC_TRIG  = _IOW(LIBERA_LOW_MAGIC,
				   LIBERA_LOW_MC_TRIG, libera_S32_t),

    LIBERA_LOW_GET_SC_TRIGGER_19= _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_SC_TRIGGER_19, libera_hw_time_t),

    LIBERA_LOW_GET_MC_TRIGGER_19= _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_MC_TRIGGER_19, libera_hw_time_t),

    LIBERA_LOW_GET_SC_TRIGGER_10= _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_SC_TRIGGER_10, libera_hw_time_t),
    LIBERA_LOW_GET_SC_TRIGGER_9= _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_SC_TRIGGER_9, libera_hw_time_t),

    LIBERA_LOW_GET_MC_TRIGGER_10= _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_MC_TRIGGER_10, libera_hw_time_t),
    LIBERA_LOW_GET_SC_TIME      = _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_SC_TIME, libera_S32_t),
    LIBERA_LOW_GET_MC_TIME      = _IOR(LIBERA_LOW_MAGIC, 
				   LIBERA_LOW_MC_TIME, libera_S32_t),
    
    /* Transfer of measured MC frequency f_MC */
    LIBERA_LOW_GET_FLMC         = _IOR(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_FLMC, libera_U32_t),
    LIBERA_LOW_SET_FLMC         = _IOW(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_FLMC, libera_U32_t),
        
    /* Current Libera time */
    LIBERA_LOW_GET_CTIME         = _IOR(LIBERA_LOW_MAGIC,
					LIBERA_LOW_CTIME, libera_Ltimestamp_t),

    /* Libera time of the last TRIGGER trigger */
    LIBERA_LOW_GET_TRIG_TRIGGER  = _IOR(LIBERA_LOW_MAGIC,
					LIBERA_LOW_TRIG_TRIGGER, 
					libera_Ltimestamp_t),

    LIBERA_LOW_GET_EVENT        = _IOR(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_EVENT, 
				       libera_U32_t),

    LIBERA_LOW_SET_EVENT        = _IOW(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_EVENT, 
				       libera_U32_t),

    LIBERA_LOW_EVENT_FLUSH      = _IOW(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_EVENTFLUSH, 
				       libera_U32_t),

    /* All DEBUG IDs have to be declared at the bottom of enum! */
#ifdef DEBUG
    LIBERA_LOW_GET_TRIGGER_BLOCKED = _IOR(LIBERA_LOW_MAGIC,
					  LIBERA_LOW_TRIGGER_BLOCK, 
					  libera_Ltimestamp_t),
    
    LIBERA_LOW_PEEK             = _IOR(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_PEEK_POKE,
				       libera_peek_poke_t),


    LIBERA_LOW_POKE             = _IOW(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_PEEK_POKE,
				       libera_peek_poke_t),

    LIBERA_LOW_EVENT_SIM        = _IOW(LIBERA_LOW_MAGIC,
				       LIBERA_LOW_EVENTSIM, 
				       libera_U32_t),
    
#endif
};

#define TRIGGER_BIT(x)     (1 << x)

#ifdef __cplusplus
}
#endif

#endif // _LIBERA_H
