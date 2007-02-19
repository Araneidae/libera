// $Id: dscd_impl.h,v 1.4 2006/11/30 15:30:46 ales Exp $

//! \file dscd_impl.h
//! Declares compensation methods in the DSC Daemon.

/*
CSPI DSC Daemon
Copyright (C) 2003-2006 Instrumentation Technologies, Slovenia

This program is licenced software; you can use it under the terms of the
Instrumentation Technologies License. You should have received a copy of the
Licence along with this program; if not, write to the Instrumentation
Technologies, Velika pot 22, 5250 Solkan, Slovenia

This program source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY to the extend permitted by applicable law.

All rights reserved.
Any copying, distribution and/or disclosure prohibited.
Copyright (C) 2006 Instrumentation Technologies
*/

#if !defined(_DSC_IMPL_H)
#define _DSC_IMPL_H

#include "dscd.h"

#ifdef __cplusplus
extern "C" {
#endif


//--------------------------------------------------------------------------
// Globals

/* Default decimation factor. */
extern size_t _DEC;
/* Default revolution frequency [Hz]. */
extern double _f_TBT;
/* Default number of TBT samples per switch position. */
extern size_t _N_TBT;
/* Default harmonic number. */
extern size_t _HARMONIC;
/* Sum of attenuators @0dBm [dB]. */
extern size_t _ATTNSUM_0dBm;
/* ADC-rate buffer peak level @0dBm [ADC count]. */
extern size_t _ADCPEAK_0dBm;
/* TBT marker delay for compensation of DDC propagation delay (in ADC samples) */
extern size_t _TBT_M_DELAY;
/* analog to digital switch propagation delay (in ADC samples) */
extern size_t _A2D_DELAY;
/* number of full sitching periods for phase compensation calculations */
extern size_t _PH_AVG;
/* _tune_offset */
extern size_t _tune_offset;
/* compensated tune offset (Y/N) */
extern size_t _comp_tune;
/* Machine Clock prescaler */
extern size_t _MC_presc;

/* FPGA R/W initialization */
extern int fpga_rw_init(unsigned long base_addr);
/* FPGA R/W cleanup */
extern int fpga_rw_cleanup();

//--------------------------------------------------------------------------
// Interface.

/* Called once at server startup. */
int init_compensation();

/* Called once at server shutdown. */
int exit_compensation();

/* Called to process a message. */
int handle_message( message *p );

/* Called by server to perform amplitude compensation (one pass). */
int compensate_amplitude();

/* Called by server to perform phase compensation (one pass). */
int compensate_phase();

/* Called by server to perform crosstalk compensation (one pass). */
int compensate_crosstalk();

/* Called by server to perform gain compensation (one pass). */
int compensate_gain();

#ifdef __cplusplus
}
#endif
#endif	// _DSC_IMPL_H
