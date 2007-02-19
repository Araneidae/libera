// $Id: dsc_fpga.h,v 1.12 2006/11/30 15:30:46 ales Exp $

//! \file dsc_fpga.c
//! Public DSC-FPGA header file.

/*
Copyright (C) 2003-2006 Instrumentation Technologies, Slovenia

This program is licenced software; you can use it under the terms of the
Instrumentation Technologies License. You should have received a copy of the
Licence along with this program; if not, write to the Instrumentation
Technologies, Velika pot 22, 5250 Solkan, Slovenia

This program source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY to the extend permitted by applicable law.

All rights reserved.
Any copying, distribution and/or disclosure prohibited.
*/

#ifndef __DSC_FPGA_H
#define __DSC_FPGA_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <syslog.h>



#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
__LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define FPGA_BASE_ADDR 0x14000000
// #define MAP_SIZE 4096UL
#define MAP_SIZE 0x00010000 // 0x00040000
#define MAP_MASK (MAP_SIZE - 1)

// this prints out detailed FPGA write operations with addresses and values
//#define __DEVEL


// FPGA addresses
#define ADC_SCOPE_CTRL			0x00008000
#define DSC_ATT_L				0x0000C008
#define DSC_ATT_H				0x0000C00C
#define ATT_BANK_SIZE			0x00000008

#define DSC_PATTERN_GEN_BASE	0x0000C800
#define DSC_PATTERN_GEN_TOP 	0x0000CFFF
#define DSC_DOUBLE_BUFF_CR		0x0000C024
#define DSC_DELAY_CR			0x0000C028
#define DSC_HIST_BUFF_MARK_CR	0x0000C030
#define	DSC_ADCRB_TRIGGER_CR	0x0000C02C
#define	DSC_LINEARIZATION_BASE	0x0000E000
#define	DSC_LINEARIZATION_TOP	0x0000E3FF
#define	DSC_HYSTERESIS_BASE		0x0000E400
#define	DSC_HYSTERESIS_TOP		0x0000E7FF
#define	DSC_PHASE_GAIN_BASE		0x0000E800
#define	DSC_PHASE_GAIN_TOP		0x0000EBFF
#define	DSC_XTALK_BASE			0x0000F000
#define	DSC_XTALK_TOP			0x0000F7FF



// limitations
#define N_POS_IN_INT			8															// number of positions in one int (32 bit)
#define MAX_SW_PATTERN			(DSC_PATTERN_GEN_TOP-DSC_PATTERN_GEN_BASE+1)*N_POS_IN_INT	// max switch pattern size
#define MAX_SW_POSITIONS		16															// number of all switch positions
#define MAX_CHANNELS			4						// number of analog channels
#define MAX_ATTS				8						// number of attenuators
#define MAX_TBT_DELAY			1023					// maximum value of TBT delay for switching pattern generator
#define MAX_ANA2DIG_DELAY		1023					// maximum value of delay between input switches and DSC chain
#define SEGMENTS				32						// number of linearization and hysteresis segments
#define	MAX_K1_K2				0x1FFFF					// maximum calculated value for K1 in K2 in phase & gain compensation
#define MAX_XTALK				0x1FFFF					// maximum calculated value for crosstalk coefficients
#define K_XTALK_UNITY			0x8000					// crosstalk unity gain (was divided by 2 not to bring DDC into overflow !)
#define PHGN_UNITY				0x10000					// phase & gain unity gain
#define MIN_ADCRB_TRIG_DELAY	0x0001					// min ADCRB trigger delay
#define MAX_ADCRB_TRIG_DELAY	0xFFFF					// max ADCRB trigger delay
#define MAX_TBT_MARKER_DELAY	0xFFFF					// max TBT marker delay


// constants
#define	CH_A			0								// channel A number
#define	CH_B			1
#define	CH_C			2
#define	CH_D			3								// channel D number
#define PI				((double) 3.14159265358979323846)// pi constant
#define NORMAL_T		0								// normal (event generated) trigger
#define FORCED_T		1								// forced trigger
#define ADCRB_EXT_TRIG	0
#define ADCRB_DSC_TRIG	1




// DSC structure
typedef struct  {
	unsigned int	positions[MAX_SW_PATTERN];
	unsigned int	no_of_positions;
	unsigned int	TBT_delay;
	unsigned int	analog_2_digital_delay;
	unsigned int	linearization_k[MAX_CHANNELS][SEGMENTS];
	unsigned int	linearization_N[MAX_CHANNELS][SEGMENTS];
	unsigned int	hysteresis_k[MAX_CHANNELS][SEGMENTS];
	unsigned int	hysteresis_N[MAX_CHANNELS][SEGMENTS];		
	float			gain[MAX_SW_POSITIONS][MAX_CHANNELS];
	float			phase[MAX_SW_POSITIONS][MAX_CHANNELS];
	float			xtalk[MAX_SW_POSITIONS][MAX_CHANNELS][MAX_CHANNELS];
	double			freq_IF;
	double			freq_samp;
	double			delta_IF;
	unsigned int 	attenuators[MAX_ATTS];
} dsc_struct;

//dsc_struct storage_struct, storage_struct_old;		// main structure where all parameters are stored and its copy that was last written to the FPGA
//int first_write;										// first write to the FPGA
//int double_buffer;									// double buffering bank selection

//----------------------------------------------------------------------------------------------------------------------------------
// Apply all settings to the FPGA
int DSC_Apply_all (void);


// Sets DSC to default values
//int DSC_init(float freq_samp, float freq_IF, int channel_order);
int DSC_init(double freq_samp, double freq_IF, int channel_order);

// Switching pattern generator settings
int DSC_Set_sw_pattern (unsigned int * positions, unsigned int pos_no);
int DSC_Set_TBT_delay (unsigned int TBT_delay);
int DSC_Set_ana2dig_delay (unsigned int ana2dig_delay);


// Phase compensation settings
// delay settable from -Pi to +Pi ???
int DSC_Set_phase (unsigned int sw_position, unsigned int channel, float delay, double delta_IF);


// Gain compensation settings
// gain settable from -2 to +2
int DSC_Set_gain (unsigned int sw_position, unsigned int channel, float gain);


//Set attenuators
// order of ATTs in the array: A1, A2, B1, B2, C1, C2, D1, D2
int DSC_set_att(unsigned char * attenuators);


// ADC Rate buffer settings (directly, no writting into structure first and applying after all changes are written);
int DSC_Set_ADCRB_trigger (unsigned int position, unsigned int delay, unsigned int trig_src, unsigned int forced);

// Wait for ADCRB trigger till it happens or timeout (given in 10ms steps)occurs
void ADCRB_wait_trigger (unsigned int maxt_x10ms);


// XTALX matrice setting
int DSC_Set_xtalk (unsigned int sw_position, unsigned int frm_channel, unsigned int to_channel, float value);


// TBT marker setting (directly, no writting into structure first and applying after all changes are written);
int DSC_Set_TBT_marker (unsigned int position, unsigned int delay);

//-------------------- Error codes -----------------------------------------------------------
#define ERR_NULL_POINTER				-1
#define ERR_TOO_MANY_POSITIONS			-2
#define ERR_INVALID_POSITION			-3
#define ERR_ANA2DIG_DELAY_TOO_LONG		-4
#define	ERR_TBT_DELAY_TOO_LONG			-5
#define PHGN_COEF_OUT_OF_RANGE			-6
#define XTALK_COEF_OUT_OF_RANGE			-7
#define ADCRB_TRIG_DELAY_OUT_OF_RANGE	-8
#define SW_POSITION_OUT_OF_RANGE		-9
#define TBT_MARKER_OUT_OF_RANGE			-10


#endif // __DSC_FPGA_H
