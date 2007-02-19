// $Id: dscd_impl.c,v 1.22 2006/12/04 10:01:52 primoz Exp $

//! \file dscd_impl.c
//! Implements different compensation methods in the DSC Daemon.

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
*/

#define _GNU_SOURCE
#include <stdlib.h>
#include <string.h> // !!!!!!!!!!!!!!!
#include <errno.h> // !!!!!!!!!!!!!!!
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>

#include "cspi.h"
#include "debug.h"

#include "dscd_impl.h"
#include "cordic.h"
#include "dsc_fpga.h"
#include "cordic_dsc.h"

//compile options (extras)
//#define ABDC

#define N_SW_POS				8						// switch positions in our sequence
#define TBT_READ_SIZE 			_N_TBT * N_SW_POS * 2	// size of TBT data to be read
#define ADC_READ_SIZE			1024					// size of ADCRB data to be read
#define ADC_RANGE				2048					// ADC positive peak value +1
#define N_ACQ_AVG				8						// averaging over x acquisitions
#define ALL_CH					4						// all channels
#define LEAVE_OUT_START			4						// number of samples we leave out at beginning of amplitude comp.
#define LEAVE_OUT_END			2						// number of samples we leave out at end of amplitude comp.
#define ADCRB_DELAY				0x400					// to be securely outside switching area
#define AMP_MIN_F				0.95
#define CG						1.64676019268469		// CORDIC gain
//#define CG					1.00000000000000		// CORDIC gain - if it is already compensated in the CORDIC function
#define HYSTER					0.25
#define MAX_HISTER				1.00					// maximum reasonable hysteresis
#define SLOW_P_CHG_LIMIT 		4
#define FAST_ATT_CHANGE	 		0.5
#define COMP_LG_RAM_FILENAME	"/tmp/dsc_lastgood.dat" // in RAM disc space
#define COMP_LASTGOOD_FILENAME	"/opt/dsc/lastgood.dat"	// in FLASH disc space
#define GAIN_FILENAME			"/opt/dsc/gain.conf"	// gain table
#define AMP_COMP_VALID			0x01					// bitmask for valid amplitude compensation coefficients
#define PHASE_COMP_VALID		0x02					// bitmask for valid phase compensation coefficients
#define FLASH_MIN_SAVE_TIME		60						// only every 60 seconds "lastgood" coefficints can be stored


// TODO: add preprocessor definitions

//--------------------------------------------------------------------------
// Globals.

// TODO: add globals here...
CSPIHENV henv;

CSPIHCON hcon_tbt = 0;							// TbT connection handle
CSPIHCON hcon_adc = 0;							// ADC connection handle
CSPI_DD_RAWATOM * DD_buffer = NULL;				// Data on Demand buffer
CSPI_ADC_ATOM * ADC_buffer = NULL;				// ADCRB

int * amp_A = NULL;								// amplitude buffers
int * amp_B = NULL;
int * amp_C = NULL;
int * amp_D = NULL;


#ifdef ABDC

	unsigned int pos_array_fixed[]=		{7,7,7,7,7,7,7,7};				// direct switch_position - ABDC
	unsigned int pos_array_rotating[]=	{3,1,0,2,14,12,13,15};			// switch 8 positions - BoSo - ABDC
	const unsigned char sw_table [] = 	{3, 2, 0, 1, 3, 1, 0, 2, 0, 2, 3, 1, 0, 1, 3, 2, 3, 2, 1, 0, 3, 1, 2, 0, 0, 2, 1, 3, 0, 1, 2, 3,
  										2, 3, 0, 1, 1, 3, 0, 2, 2, 0, 3, 1, 1, 0, 3, 2, 2, 3, 1, 0, 1, 3, 2, 0, 2, 0, 1, 3, 1, 0, 2, 3};
	const unsigned char de_sw_table [] = {2, 3, 1, 0, 2, 1, 3, 0, 0, 3, 1, 2, 0, 1, 3, 2, 3, 2, 1, 0, 3, 1, 2, 0, 0, 2, 1, 3, 0, 1, 2, 3, 
										2, 3, 0, 1, 2, 0, 3, 1, 1, 3, 0, 2, 1, 0, 3, 2, 3, 2, 0, 1, 3, 0, 2, 1, 1, 2, 0, 3, 1, 0, 2, 3};	
#else

	unsigned int pos_array_fixed[]=		{3,3,3,3,3,3,3,3};				// direct switch_position - ABCD
	unsigned int pos_array_rotating[]=	{3,7,15,11,0,4,12,8};			// switch	positions - BoSo - ABCD
	//unsigned int pos_array_rotating[]={3,12,15,0,3,12,15,0};			// switch	positions - Peter - ABCD	
	const unsigned char sw_table [] = 	{3, 2, 1, 0, 3, 1, 2, 0, 0, 2, 1, 3, 0, 1, 2, 3, 3, 2, 0, 1, 3, 1, 0, 2, 0, 2, 3, 1, 0, 1, 3, 2,
										2, 3, 1, 0, 1, 3, 2, 0, 2, 0, 1, 3, 1, 0, 2, 3, 2, 3, 0, 1, 1, 3, 0, 2, 2, 0, 3, 1, 1, 0, 3, 2};
	const unsigned char de_sw_table [] = {3, 2, 1, 0, 3, 1, 2, 0, 0, 2, 1, 3, 0, 1, 2, 3, 2, 3, 1, 0, 2, 1, 3, 0, 0, 3, 1, 2, 0, 1, 3, 2,
										3, 2, 0, 1, 3, 0, 2, 1, 1, 2, 0, 3, 1, 0, 2, 3, 2, 3, 0, 1, 2, 0, 3, 1, 1, 3, 0, 2, 1, 0, 3, 2};
#endif


#define ATT1 26		// !!!!!!!!!!!!!!!!!
#define ATT2 18		// !!!!!!!!!!!!!!!!!						 		
#define MAX_ATT 31

#define MAX_INP_POWER	+30				// maximum input power in the table
#define MIN_INP_POWER	-100			// minimum inp power in the table
#define ATT_TABLE_OFFS	-(MIN_INP_POWER)// lowest input power at position 0
#define MULT_EQU1		0x4000			// multiplication by 1 for phase shift
#define Q_SHIFT 		14				// shift to return to short format
#define K_F_AGC 		1.0				// fast filter constant for AGC
#define K_S_AGC 		0.125			// slow filter constant for AGC

// Phase compensation
#define TBT_READ_SIZE_PHASE 			_N_TBT * N_SW_POS * (_PH_AVG+1)			// size of TBT data to be read, for phase compensation
#define THREE_HALF_PI					((double)1.500 * PI)
#define TWO_PI							((double)2.000 * PI)
#define INDEX_LOW_PART					0.2
#define INDEX_HIGH_PART					0.8
#define INDEX_LOW						round(INDEX_LOW_PART * _N_TBT)
#define INDEX_HIGH						round(INDEX_HIGH_PART * _N_TBT)
#define CONST_FOR_AVERAGE_PARAM			0.5
#define CONST_FOR_AVERAGE				(int) round(CONST_FOR_AVERAGE_PARAM * _N_TBT)
#define CORDIC_LONG_TO_FLOAT_NORM   	0.0000000000000000008673617				// 1/1152921504606847000
#define PHASE_INDEX_INCREMENT			round(sizeof(CSPI_DD_RAWATOM)/sizeof(int))


// Global variables

//-------------------------------------------------------------------------------------------------
// Phase compensation variables

double * angle_present = NULL;					
double * delta_angle_present = NULL;					
double * angle_diff = NULL;
double * angle_temp = NULL;
double * new_angle_array = NULL;
double * old_angle_array = NULL;
int error_on_atan2 = 0;


int phase_test_counter = 3;

struct DSC_COMPPARAMS comp_dbase [MAX_INP_POWER - MIN_INP_POWER + 1];

//-------------------------------------------------------------------------------------------------
// AGC variables

unsigned char atts[]={ATT1, ATT2, ATT1, ATT2, ATT1, ATT2, ATT1, ATT2};
unsigned char max_atts[]={MAX_ATT, MAX_ATT, MAX_ATT, MAX_ATT, MAX_ATT, MAX_ATT, MAX_ATT, MAX_ATT };

struct {
	unsigned int att1;
	unsigned int att2;
	float hysteresis;
	unsigned int time;
	
} ATT_table [MAX_INP_POWER - MIN_INP_POWER + 1];

int	highest_att_entry = MIN_INP_POWER,
	lowest_att_entry = MAX_INP_POWER;

int input_level = MAX_INP_POWER+1;												// global input level
int old_input_level = 0xff;
int delay_K1 = 0;
int delay_K2 = 0;
float peak_filtered = ADC_RANGE;												// like ADCs were saturated at the beginning
int ATT_sum = 0;																// sum of ATTs is not known yet, therefore assume worst case
int AGC_skip_N = 0;																// samples to be skipped for proper Q calculation 

			

//-------------------------------------------------------------------------------------------------
// common variables

double f_samp = 117440042.7350;			
double f_IF = 29893829.0598;

unsigned int DSC_switch = CSPI_SWITCH_DIRECT;
//unsigned int old_DSC_switch = CSPI_SWITCH_AUTO;								//just to be different!
unsigned int old_DSC_switch = 0xffff;											//just to be different from everything legal


unsigned int DSC_mode = CSPI_DSC_OFF;
unsigned int old_DSC_mode = CSPI_DSC_AUTO;										//just to be different!

unsigned int AGC_mode = CSPI_AGC_MANUAL;
#define AGC_mode_mask		0x00000001
#define AGC_locked			0x00000002

int * current_pattern = NULL;

int lastgood_save_time;															// recently saved to the FLASH


//--------------------------------------------------------------------------
// Local decls.

// TODO: add local function declarations with short descriptions here...

// called on DSCD_SET_AGC message
int set_agc( int arg );
// called on DSCD_GET_AGC message
int get_agc( int *arg );

// called on DSCD_SET_DSC message
int set_dsc( int arg );
// called on DSCD_GET_DSC message
int get_dsc( int *arg );

// called on DSCD_SET_GAIN message
int set_gain( int arg );
// called on DSCD_GET_GAIN message
int get_gain( int *arg );

// called on DSCD_SET_SWITCH message
int set_switch( int arg );
// called on DSCD_GET_SWITCH message
int get_switch( int *arg );

// called from most of the "set_..." functions
void DSC_apply_new_settings();

// called from set_agc function
int AGC_read_table(void);


// reads last good amplitude or phase compensation coefficients from file
int write_comp_coeff (unsigned char *);
// writes last good amplitude or phase compensation coefficients from file
int read_comp_coeff (unsigned char *);


// PHASE COMPENSATION
void inline atan2_array(int * y_array, int * x_array, long array_start, long array_finish, double *atan2_output);
void inline correct_phase_to_absolute_value(double *angle_inout, long array_start, long array_finish);
double inline mean_value(double *data_input, long array_start, long array_finish);
void inline calc_angle_diff(int * input_1, int * input_2, int * input_3, int * input_4, double * angle_out, double * angle_temp, long array_start, long array_finish);

//**************************************************************************************************	
int init_compensation()
//**************************************************************************************************	
{

	unsigned char level = 0;
	unsigned char sw_pos = 0;
	unsigned char channel = 0;
	float AGC_angle = 0;
	
	if ( fpga_rw_init(FPGA_BASE_ADDR) ) FATAL;

	int rc = cspi_allochandle( CSPI_HANDLE_ENV, 0, &henv );
	if (CSPI_OK != rc) return -1;

	rc = cspi_allochandle( CSPI_HANDLE_CON, henv, &hcon_tbt );
	if (CSPI_OK != rc) return -1;

	CSPI_CONPARAMS params;
	params.mode = CSPI_MODE_DD;

	rc = cspi_setconparam( hcon_tbt, &params, CSPI_CON_MODE );
	if (CSPI_OK != rc) return -1;

	rc = cspi_connect( hcon_tbt );
	if (CSPI_OK != rc) return -1;

	rc = cspi_allochandle( CSPI_HANDLE_CON, henv, &hcon_adc );
	if (CSPI_OK != rc) return -1;

	params.mode = CSPI_MODE_ADC;
	rc = cspi_setconparam( hcon_adc, &params, CSPI_CON_MODE );
	if (CSPI_OK != rc) return -1;

	rc = cspi_connect( hcon_adc );
	if (CSPI_OK != rc) return -1;

	// allocate common TBT acquisition buffer (highest size of both)
	if (TBT_READ_SIZE >= TBT_READ_SIZE_PHASE) {
			DD_buffer = malloc(TBT_READ_SIZE * sizeof(CSPI_DD_RAWATOM));
			if (DD_buffer == NULL)	return -1;
	}
	else {
			DD_buffer = malloc(TBT_READ_SIZE_PHASE * sizeof(CSPI_DD_RAWATOM));
			if (DD_buffer == NULL)	return -1;
	}

	angle_present = malloc(TBT_READ_SIZE_PHASE * sizeof(double));
	if (angle_present == NULL)	return -1;
	delta_angle_present = malloc(TBT_READ_SIZE_PHASE * sizeof(double));
	if (delta_angle_present == NULL)	return -1;
	
	angle_diff = malloc( (INDEX_HIGH - INDEX_LOW + 1) * sizeof(double));		
	if (angle_diff == NULL)	return -1;
	angle_temp = malloc( (INDEX_HIGH - INDEX_LOW + 1) * sizeof(double));		
	if (angle_temp == NULL)	return -1;
	new_angle_array = malloc( CONST_FOR_AVERAGE * sizeof(double));		
	if (new_angle_array == NULL)	return -1;
	old_angle_array = malloc( CONST_FOR_AVERAGE * sizeof(double));		
	if (old_angle_array == NULL)	return -1;


	// allocate ADCRB acquisition buffer
	ADC_buffer = malloc(ADC_READ_SIZE * sizeof(CSPI_ADC_ATOM));
	if (ADC_buffer == NULL)	return -1;

	// allocate amplitude buffer
	amp_A = malloc (TBT_READ_SIZE * sizeof (int));
	if (amp_A == NULL) return -1;
	
	amp_B = malloc (TBT_READ_SIZE * sizeof (int));
	if (amp_B == NULL) return -1;	
	
	amp_C = malloc (TBT_READ_SIZE * sizeof (int));
	if (amp_C == NULL) return -1;	

	amp_D = malloc (TBT_READ_SIZE * sizeof (int));
	if (amp_D == NULL) return -1;

	// calculate VCXO and NCO frequencies (including detune)
	f_samp = _f_TBT *_DEC * (1 + (double) _tune_offset / (_DEC * _MC_presc));
	f_IF = _f_TBT * _HARMONIC - f_samp * (_HARMONIC / _DEC);

		
#ifdef ABDC	
	DSC_init(f_samp, f_IF, 1);
#else
	DSC_init(f_samp, f_IF, 0);
#endif	

	AGC_skip_N = floor(f_samp /f_IF/4);										// samples to be skipped to get base sample for Q component
	AGC_angle = PI/2 - AGC_skip_N * f_IF/f_samp * 2 * PI;					// remaining angle to be set with the delay cell
	delay_K1 = (int) ((-sin(AGC_angle) / tan(2*PI*f_IF/f_samp) + cos(AGC_angle)) * MULT_EQU1);
	delay_K2 = (int) (sin (AGC_angle) / sin(2*PI*f_IF/f_samp) * MULT_EQU1);
	
	_LOG_DEBUG("f_samp: %f", f_samp);
	_LOG_DEBUG("f_IF: %f", f_IF);
	_LOG_DEBUG("AGC_skip_N: %d", AGC_skip_N);
	_LOG_DEBUG("AGC_angle: %f deg.", AGC_angle * 180/PI);
	_LOG_DEBUG("K1: %d", delay_K1);
	_LOG_DEBUG("K2: %d",delay_K2);

	#ifdef ABDC
		set_switch(7);
	#else
		set_switch(3);
	#endif
	
	DSC_Set_TBT_marker (current_pattern[0], _TBT_M_DELAY);								// marker to first position in array
	DSC_Set_sw_pattern(current_pattern, N_SW_POS);										// initially no switching
	DSC_Set_ADCRB_trigger (current_pattern[0], ADCRB_DELAY, ADCRB_EXT_TRIG, NORMAL_T );	// set external ADCRB trigger

	// read amplitude compensation from database or initialize to unity values	
	if (read_comp_coeff(COMP_LG_RAM_FILENAME) == 1) {									// if there is no ampcomp file on the RAM disk
		syslog(LOG_ERR, "%s not found", COMP_LG_RAM_FILENAME);
		
		if (read_comp_coeff(COMP_LASTGOOD_FILENAME) == 1) {								// if there is no "lastgood" file on the FLASH disk
			syslog(LOG_ERR, "%s not found", COMP_LASTGOOD_FILENAME);
			
			for (level = 0; level <= (MAX_INP_POWER - MIN_INP_POWER); level ++)
				for (sw_pos = 0; sw_pos < MAX_SW_POSITIONS; sw_pos++)					// initialize the whole database
					for (channel = 0; channel < ALL_CH; channel++) {
						comp_dbase[level].ampl[sw_pos][channel] = 1.0000;				// with unity value
						comp_dbase[level].phase[sw_pos][channel] = 0.0000;				// with zero angle					
						comp_dbase[level].status = 0;									// not calculated yet										
					}
		}
	}
	
	// read AGC table (gain scheme)
	if (AGC_read_table() == -1)															// operation without proper AGC table not allowed
		return -1;

	input_level = highest_att_entry;
	DSC_apply_new_settings();
	
	return 0;
}


//**************************************************************************************************	
int exit_compensation()
//**************************************************************************************************	
{
	if (hcon_tbt) {
		cspi_disconnect( hcon_tbt );
		cspi_freehandle( CSPI_HANDLE_CON, hcon_tbt );
	}
	if (hcon_adc) {
		cspi_disconnect( hcon_adc );
		cspi_freehandle( CSPI_HANDLE_CON, hcon_adc );
	}
	cspi_freehandle( CSPI_HANDLE_ENV, henv );
	
	if (DD_buffer)
		free (DD_buffer);
		
	if (angle_present)
		free (angle_present);

	if (delta_angle_present)
		free (delta_angle_present);

	if (angle_diff)
		free (angle_diff);

	if (angle_temp)
		free (angle_temp);

	if (new_angle_array)
		free (new_angle_array);

	if (old_angle_array)
		free (old_angle_array);


	if (ADC_buffer)
		free (ADC_buffer);
		
	if (amp_A)
		free (amp_A);

	if (amp_B)
		free (amp_B);
		
	if (amp_C)
		free (amp_C);
		
	if (amp_D)
		free (amp_D);
	
	
	#ifdef ABDC
		set_switch(7);
	#else
		set_switch(3);
	#endif
	
	
	write_comp_coeff(COMP_LG_RAM_FILENAME);
	
	DSC_Set_TBT_marker (current_pattern[0], _TBT_M_DELAY);								// marker to first position in array
	DSC_Set_sw_pattern(current_pattern, N_SW_POS);										// initially no switching
	DSC_Set_ADCRB_trigger (current_pattern[0], ADCRB_DELAY, ADCRB_EXT_TRIG, NORMAL_T );	// set external ADCRB trigger
	
	DSC_Apply_all ();
	
	if ( fpga_rw_cleanup() ) FATAL;

	return 0;
}


//**************************************************************************************************	
int handle_message( message *p )
//**************************************************************************************************	
{
	_LOG_DEBUG("received message "
	          "{magic=%d, type=%d, val=%d, pid=%d, status=%d}",
	          p->magic, p->type, p->val, p->pid, p->status);

	switch ( p->type ) {

		case DSCD_SET_AGC:
			return set_agc(p->val);

		case DSCD_GET_AGC:
			return get_agc(&p->val);

		case DSCD_SET_DSC:
			return set_dsc(p->val);

		case DSCD_GET_DSC:
			return get_dsc(&p->val);

		case DSCD_SET_GAIN:
			return set_gain(p->val);

		case DSCD_GET_GAIN:
			return get_gain(&p->val);

		case DSCD_SET_SWITCH:
			return set_switch(p->val);

		case DSCD_GET_SWITCH:
			return get_switch(&p->val);

			/*
		case DSCD_SET_COMPDATA:
			return set_compdata(p->val);

		case DSCD_GET_COMPDATA:
			return get_compdata(&p->val);
			*/

	}
	return -1;	// unknown message type
}


//**************************************************************************************************	
int set_agc( int arg )
//**************************************************************************************************	
{
	_LOG_DEBUG("%s(%d)", __FUNCTION__, arg);
	switch (arg) {
		case CSPI_AGC_MANUAL:	
			AGC_mode = CSPI_AGC_MANUAL;											// report new mode
			DSC_apply_new_settings();
			return 0;
				
		case CSPI_AGC_AUTO:		
			AGC_mode = CSPI_AGC_AUTO;											// report new mode  (also not locked yet!)
			DSC_apply_new_settings();			
			return 0;
			
		default: return -1;														// any other unknown option
	}
	
	return -1;
}


//**************************************************************************************************	
int get_agc( int *arg )
//**************************************************************************************************	
{
	_LOG_DEBUG("%s(%p)", __FUNCTION__, arg);
	* arg = (AGC_mode & AGC_mode_mask);
	return 0;	
}


//**************************************************************************************************	
int set_dsc( int arg )
//**************************************************************************************************	
{
struct timeval current_time;
struct timezone tz;
	
	_LOG_DEBUG("%s(%d)", __FUNCTION__, arg);
	
	switch (arg) {
		case CSPI_DSC_OFF:
			DSC_mode = CSPI_DSC_OFF;
			DSC_apply_new_settings();
			return 0;
			
		case CSPI_DSC_UNITY:
			DSC_mode = CSPI_DSC_UNITY;
			DSC_apply_new_settings();
			return 0;
			
		case CSPI_DSC_AUTO:
			DSC_mode = CSPI_DSC_AUTO;												// switch it on
			set_switch (CSPI_SWITCH_AUTO);											// this also requires switching
			DSC_apply_new_settings();
			return 0;
			
		case CSPI_DSC_SAVE_LASTGOOD:												// saves current coefficient to FLASH
			if (gettimeofday( &current_time, &tz ) == 0) {							// current time correctly read
				if (current_time.tv_sec - lastgood_save_time > FLASH_MIN_SAVE_TIME) {// already allowed to write
					lastgood_save_time = current_time.tv_sec;						// remember the time of last write attempt (regardless of the success)
					return (write_comp_coeff(COMP_LASTGOOD_FILENAME));				// successfully written
				}
				else
					return (-EAGAIN);												// not allowed to write yet
			}
			
			else																	// current time not correctly read
				return (-1);														// try again message
			
		default:
			return -1;	
	}
}


//**************************************************************************************************	
int get_dsc( int *arg )
//**************************************************************************************************	
{
	_LOG_DEBUG("%s(%p)", __FUNCTION__, arg);
	* arg = DSC_mode;
	return 0;
}


//**************************************************************************************************	
int set_gain( int arg )
//**************************************************************************************************	
{
	
	int rc = 0;
	
	_LOG_DEBUG("%s(%d)", __FUNCTION__, arg);
	_LOG_DEBUG("highest_att_entry: %d",highest_att_entry);
	_LOG_DEBUG("lowest_att_entry: %d",lowest_att_entry);	
	_LOG_DEBUG("AGC auto: %d",AGC_mode);		

		
	if ((arg  > highest_att_entry) || (arg  < lowest_att_entry)) { 				// desired level outside ATT table limits					
		arg = highest_att_entry;												// set to highest known level
		rc = -1;
	}

	_LOG_DEBUG("Manual level: %d",input_level);

	if ((AGC_mode & AGC_mode_mask) == CSPI_AGC_MANUAL) {						// if in manual mode
		input_level = arg;														// also store		
		DSC_apply_new_settings();												// apply settings
	}
	else
		rc = -1;
	
	_LOG_DEBUG("set gain return code: %d",rc);	
	return rc;
	
}


//**************************************************************************************************	
int get_gain( int *arg )
//**************************************************************************************************	
{
	_LOG_DEBUG("%s(%p)", __FUNCTION__, arg);
	* arg = input_level;
	return 0;
}


//**************************************************************************************************	
int set_switch( int arg )
//**************************************************************************************************	
{
	int ret_val = 0;
	unsigned char i;
	
	_LOG_DEBUG("%s(%d)", __FUNCTION__, arg);	
	
	
	// requested MANUAL switching mode			
	if ((arg >= 0) && (arg < MAX_SW_POSITIONS)) {								// manual switch position

		for (i=0; i < N_SW_POS; i++)											// write the whole pattern with the same position
			pos_array_fixed[i] = arg;
	
		current_pattern = pos_array_fixed;										// use fixed pattern
		
		if (DSC_mode == CSPI_DSC_AUTO)											// if currently automatic switching is running
			set_dsc(CSPI_DSC_OFF);												// DSC_OFF mode is now selected
	}

	// requested AUTO switching mode
	else {
		arg = CSPI_SWITCH_AUTO;
		current_pattern = pos_array_rotating;
	}

/*	if (arg != DSC_switch) {													// switch mode changed
		DSC_Set_sw_pattern (current_pattern, N_SW_POS);							// turn on switching
		DSC_Set_TBT_marker (current_pattern[0], _TBT_M_DELAY);					// marker to first position in array
		DSC_Apply_all ();
		DSC_switch = arg;
		_LOG_DEBUG("Switch mode %d applied", DSC_switch);
	} */
	
	DSC_switch = arg;
	DSC_apply_new_settings();
	
	return ret_val;
}


//**************************************************************************************************	
int get_switch( int *arg )
//**************************************************************************************************	
{
	_LOG_DEBUG("%s(%p)", __FUNCTION__, arg);
	* arg = DSC_switch;
	return 0;	
	
}


//**************************************************************************************************	
int compensate_amplitude()
//**************************************************************************************************	
{
int n_read = 0;
int i, sw_pos, sample, channel, n_succ_read = 0;
unsigned long long offset = 0;

double avg_sw_ampl_geo[ALL_CH] = {1.0, 1.0, 1.0, 1.0};							// average geometric amplitude per channel
long long avg_sw_amp_pos [N_SW_POS][ALL_CH];									// average switched amplitude (position dependant)
double avg_ch_coeff_geo[ALL_CH];												// average geometrical coefficients per channel

long long amplitude_sum;														// temporary result
unsigned int n_mark;															// number of consecutive markers
int marker;																		// marker position
int res;	
int amplitude = 0;																// amplitude calculated from Is & Qs

// initial activities
	if ( DSC_mode != CSPI_DSC_AUTO)												// if DSC is not in auto mode, finish here
		return 0;

	_LOG_DEBUG(">>>>>>>>>>>>>>>> compensating amplitudes <<<<<<<<<<<<<<<<<<<<<");
	
	for (channel = 0; channel < ALL_CH; channel++) {							// clear buffers for average values
		for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos ++)
			avg_sw_amp_pos [sw_pos][channel] = 0;			
	}

// TBT acquisition
	for (i = 0; i < N_ACQ_AVG; i++) {											// perform required number of ACQ for averaging
		_LOG_DEBUG("----> acquiring %d atoms of data...", TBT_READ_SIZE );
		cspi_seek( hcon_tbt, &offset, CSPI_SEEK_ST );
		res = cspi_read_ex(hcon_tbt, DD_buffer, TBT_READ_SIZE, &n_read, 0 );

		if (res == CSPI_OK) {
//			_LOG_DEBUG("----> %d samples acquired...", n_read);
			if (n_read == TBT_READ_SIZE) {										// only if required number of atoms was read
				for (sample = 0; sample < TBT_READ_SIZE; sample ++) {
				
					// amplitude calculation
					amp_A[sample] = cordic_amp(DD_buffer[sample].cosVa >> 2, DD_buffer[sample].sinVa >> 2 );
					amp_B[sample] = cordic_amp(DD_buffer[sample].cosVb >> 2, DD_buffer[sample].sinVb >> 2 );
					amp_C[sample] = cordic_amp(DD_buffer[sample].cosVc >> 2, DD_buffer[sample].sinVc >> 2 );
					amp_D[sample] = cordic_amp(DD_buffer[sample].cosVd >> 2, DD_buffer[sample].sinVd >> 2 );

					
					// simple (moving average) filter, Kf = 0.5
					if (sample >0) {
						amp_A[sample] = amp_A[sample]/2 + amp_A[sample-1]/2;
						amp_B[sample] = amp_B[sample]/2 + amp_B[sample-1]/2;
						amp_C[sample] = amp_C[sample]/2 + amp_C[sample-1]/2;
						amp_D[sample] = amp_D[sample]/2 + amp_D[sample-1]/2;
					}
				}

				// search for marker
				sample = 0; marker = -1; n_mark = 0;
				while ((sample < TBT_READ_SIZE / 2 + _N_TBT) && (marker < 0)) {
					if (DD_buffer[sample].cosVa & 0x00000001)
						n_mark++;												// increase number of found markers
					else	
						n_mark = 0;												// start searching from the beginning
					if (n_mark >= _N_TBT)										// if this is already complete marker block
						marker = sample -_N_TBT + 1;							// position to the beginning of marker block
					sample ++;
				}
				
				// check if marker was found --> process data
				if (marker >= 0	) {
					_LOG_DEBUG("----> marker found at %d: ",marker );					
					n_succ_read ++;												// increase number of actual successful acquisitions
					for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos++)				// through all switch positions
						for (channel = 0; channel < ALL_CH; channel++) {		// through all channels
//							_LOG_DEBUG("----> amp_avg   sw_pos: %d  channel %d", sw_pos, channel);
							amplitude_sum = 0;									// amplitude sum for this channel @ current position is 0 in this moment
																				// for all selected samples
							for (sample = marker + sw_pos * _N_TBT + LEAVE_OUT_START;
								 sample < marker + (sw_pos+1) * _N_TBT - LEAVE_OUT_END;
								 sample ++) {								 	
								 	
								switch (channel) {
									case 0: amplitude = amp_A[sample]; break;
									case 1: amplitude = amp_B[sample]; break;
									case 2: amplitude = amp_C[sample]; break;
									case 3: amplitude = amp_D[sample]; break;
									default: amplitude = 0;
									
								}
								amplitude_sum += amplitude;
							}
							
							// do the averaging over single position
							amplitude_sum = amplitude_sum / (_N_TBT - LEAVE_OUT_START - LEAVE_OUT_END);
							avg_sw_amp_pos [sw_pos][channel] += amplitude_sum;
						}
				}
			}
		}
		
		else {
			_LOG_ERR("cspi_read_ex: %s: %s", cspi_strerror(res), strerror(errno));
		}
	}

    long long TotalAmplitude = 0;
    for (channel = 0; channel < ALL_CH; channel++)
        for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos ++)
            TotalAmplitude += avg_sw_amp_pos [sw_pos][channel];
    /* Check for amplitude threshold. */
    if (TotalAmplitude < 200000000)
        /* Too little power: don't do compensation after all. */
        return 0;

// actual compensation and channel mapping
	if (n_succ_read) {															// at list one acquisition must be performed
		
		for (channel = 0; channel < ALL_CH; channel++) {						// through all channels		
			for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos++)						// through all switch positions
				avg_sw_ampl_geo[channel] *= (double) avg_sw_amp_pos [sw_pos][channel];
			
			if (avg_sw_ampl_geo[channel] > 0)									// calculate geometric average	(N-th root)		
				avg_sw_ampl_geo[channel] = exp(log (avg_sw_ampl_geo[channel]) / N_SW_POS);
		}
		
		// calculation of correction coefficients using data from last acquisition
		for (channel = 0; channel < ALL_CH; channel++) {						// through all channels
			for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos++) {						// through all switch positions
				// update of compensation coefficients with newly calculated ones
				// (in switching sequence index and ADC channel order - not in absolute position and RF chain !!!)
				comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel] *= (avg_sw_ampl_geo [channel] / (double) avg_sw_amp_pos [sw_pos][channel]);
				if (comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel] > 1.99)		// protection against escape and oscillation
					comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel] =1.99;
				comp_dbase [ATT_TABLE_OFFS + input_level].status |= AMP_COMP_VALID;
				_LOG_DEBUG(" pos: %d  channel: %d  coeff: %f", sw_pos, channel, comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel]);
			}
		}

		// calculation of geometric average of coefficients for separate channels
		for (channel = 0; channel < ALL_CH; channel++) { 						// through all channels
			avg_ch_coeff_geo[channel] = 1.00;									// start value
			for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos++)						// through all switch positions
				avg_ch_coeff_geo[channel] *= comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel];
		}
		
		
		// normalize the coefficients
		for (channel = 0; channel < ALL_CH; channel++) { 						// through all channels
			if (avg_ch_coeff_geo[channel] > 0)									// calculate geometric average
				avg_ch_coeff_geo[channel] = exp(log (avg_ch_coeff_geo[channel]) / N_SW_POS);
				
			for (sw_pos = 0; sw_pos < N_SW_POS; sw_pos++) {						// through all switch positions
				comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel] /= avg_ch_coeff_geo[channel];
				DSC_Set_gain (current_pattern[sw_pos],
							 sw_table[current_pattern[sw_pos] * ALL_CH + channel],
							 comp_dbase [ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel]);
			}
		}
		
		// apply coefficients to FPGA
		DSC_Apply_all();

	}
	
	return 1;
}

//**************************************************************************************************	
int compensate_phase()
//**************************************************************************************************	
{
	
	
int n_read = 0;
int sw_pos=0, channel=0, sample;
unsigned long long offset = 0;
int res;	
unsigned int n_mark;															// number of consecutive markers
int marker;																		// marker position
long array_start_01, array_finish_01, ppp;
double avg_angle=0;
long data_analysis_index_start=0, data_analysis_index_finish=0;
double angle_matrix[N_SW_POS][ALL_CH], new_full_angle=0;
int kkk, nnn;	
double avg_angle_diff_02, avg_angle_diff_03, avg_angle_diff_04, avg_angle_diff_01;
long indx_start, indx_finish;
double delta_IF_calc=0;

int * input_ptr_01 = NULL;
int * input_ptr_02 = NULL;
int * input_ptr_03 = NULL;
int * input_ptr_04 = NULL;


		_LOG_DEBUG(">>>>>>>>>>>>>>>> compensating phase <<<<<<<<<<<<<<<<<<<<<");
		if ( DSC_mode != CSPI_DSC_AUTO)											// if DSC is not in auto mode, finish here
				return 0;
				
		_LOG_DEBUG("----> acquiring %d atoms of data...", TBT_READ_SIZE_PHASE );
		cspi_seek( hcon_tbt, &offset, CSPI_SEEK_ST );
		res = cspi_read_ex(hcon_tbt, DD_buffer, TBT_READ_SIZE_PHASE, &n_read, 0 );

		if ((res == CSPI_OK)&& (n_read == TBT_READ_SIZE_PHASE)) {
			 	
				// search for marker
				sample = 0; marker = -1; n_mark = 0;
				while ((sample < _N_TBT*N_SW_POS * 4) && (marker < 0)) {

						if (DD_buffer[sample].cosVa & 0x00000001) 	
								n_mark++;										// increase number of found markers
						else	
								n_mark = 0;										// start searching from the beginning
								
						if (n_mark == _N_TBT){									// if this is already complete marker block
								marker = sample -_N_TBT + 1;					// position to the beginning of marker block
								
								if ( marker < (_N_TBT+9) ){						// value 9 is added since it is smaller than the lowest existing _N_TBT value (it is 10 for "desy_sr_900")
										n_mark = 0;
										marker = -1;
								}
						}
						sample ++;
				}
				//return 0;  //!!!!!!!!!!!!!!!!!!!!!!
				// check if marker was found --> process data
				if (marker >= 0	) {
						_LOG_DEBUG("----> marker found at %d: ",marker );
						
			 			error_on_atan2 = 0;
						array_start_01 = (long) (marker-floor(0.75*_N_TBT));
						array_finish_01 = (long) (marker + ceil(0.25*_N_TBT) + _PH_AVG*_N_TBT*N_SW_POS - 1 - _N_TBT);
						
						input_ptr_01 = &DD_buffer[array_start_01].cosVa;
						input_ptr_02 = &DD_buffer[array_start_01].sinVa;
						atan2_array(input_ptr_02, input_ptr_01, 0, array_finish_01-array_start_01, angle_present);
						correct_phase_to_absolute_value(angle_present, 0, array_finish_01-array_start_01);
						avg_angle = ((double) angle_present[array_finish_01-array_start_01] - angle_present[0]) / (array_finish_01-array_start_01);

					#if DEVEL
						syslog(LOG_INFO, "AVG_ANGLE: %5.10f ", avg_angle);
					#endif // DEVEL
						
						data_analysis_index_start = marker + INDEX_LOW;
						data_analysis_index_finish = marker + INDEX_HIGH;
						for(kkk=0; kkk<N_SW_POS; kkk++)
								for(nnn=0; nnn<4; nnn++){
										angle_matrix[kkk][nnn] = 0;
						}
						input_ptr_01 = &DD_buffer[array_start_01].cosVa;
						input_ptr_02 = &DD_buffer[array_start_01].sinVa;
						if ( (  (int)*input_ptr_01==0  ) && (  (int)*input_ptr_02==0  ) )  {
								new_full_angle = 0;
								error_on_atan2 = 1;	
						}
						else{
								new_full_angle = (double) cordic_DSC_phase((int) *input_ptr_01, (int) *input_ptr_02) * CORDIC_LONG_TO_FLOAT_NORM;
						}
						
						for(ppp=0; ppp<N_SW_POS; ppp++){

								input_ptr_01 = &DD_buffer[data_analysis_index_start].cosVa;
								input_ptr_02 = &DD_buffer[data_analysis_index_start].sinVa;
								input_ptr_03 = &DD_buffer[data_analysis_index_start].cosVb;
								input_ptr_04 = &DD_buffer[data_analysis_index_start].sinVb;
								calc_angle_diff(input_ptr_01, input_ptr_02, input_ptr_03, input_ptr_04, angle_diff, angle_temp, 0, (data_analysis_index_finish-data_analysis_index_start) );
								avg_angle_diff_02 = mean_value(angle_diff, 0, (data_analysis_index_finish - data_analysis_index_start) );
								
								input_ptr_01 = &DD_buffer[data_analysis_index_start].cosVa;
								input_ptr_02 = &DD_buffer[data_analysis_index_start].sinVa;
								input_ptr_03 = &DD_buffer[data_analysis_index_start].cosVc;
								input_ptr_04 = &DD_buffer[data_analysis_index_start].sinVc;
								calc_angle_diff(input_ptr_01, input_ptr_02, input_ptr_03, input_ptr_04, angle_diff, angle_temp, 0, (data_analysis_index_finish-data_analysis_index_start) );
								avg_angle_diff_03 = mean_value(angle_diff, 0, (data_analysis_index_finish - data_analysis_index_start) );

								input_ptr_01 = &DD_buffer[data_analysis_index_start].cosVa;
								input_ptr_02 = &DD_buffer[data_analysis_index_start].sinVa;
								input_ptr_03 = &DD_buffer[data_analysis_index_start].cosVd;
								input_ptr_04 = &DD_buffer[data_analysis_index_start].sinVd;
								calc_angle_diff(input_ptr_01, input_ptr_02, input_ptr_03, input_ptr_04, angle_diff, angle_temp, 0, (data_analysis_index_finish-data_analysis_index_start) );
								avg_angle_diff_04 = mean_value(angle_diff, 0, (data_analysis_index_finish - data_analysis_index_start) );

								
								indx_start = _N_TBT * (ppp+1);
								indx_finish = _N_TBT * (ppp+1) + CONST_FOR_AVERAGE - 1;

								input_ptr_01 = &DD_buffer[array_start_01 + indx_start - 1].cosVa;
								input_ptr_02 = &DD_buffer[array_start_01 + indx_start - 1].sinVa;
								atan2_array(input_ptr_02, input_ptr_01, 0, (CONST_FOR_AVERAGE-1), old_angle_array);
								correct_phase_to_absolute_value(old_angle_array, 0, (CONST_FOR_AVERAGE-1) );
								
								new_angle_array[0] = new_full_angle + avg_angle * (indx_start-1);
								for(kkk=1; kkk<CONST_FOR_AVERAGE; kkk++){
										new_angle_array[kkk] = new_angle_array[kkk-1] + avg_angle;
								}
								correct_phase_to_absolute_value(new_angle_array, 0, (CONST_FOR_AVERAGE-1) );
								
								for(kkk=0; kkk<CONST_FOR_AVERAGE; kkk++){
										new_angle_array[kkk] = new_angle_array[kkk] - old_angle_array[kkk];
								}
								avg_angle_diff_01 = mean_value(new_angle_array, 0, (CONST_FOR_AVERAGE-1) );

								
								while(avg_angle_diff_01 > TWO_PI) {
										avg_angle_diff_01 = avg_angle_diff_01 - TWO_PI;
								}
								while(avg_angle_diff_01 < (-TWO_PI) ) {
										avg_angle_diff_01 = avg_angle_diff_01 + TWO_PI;
								}

								if (ppp==(N_SW_POS-1)) {
										avg_angle_diff_01 = 0;	
								}

								angle_matrix[ppp][0] = avg_angle_diff_01;
								angle_matrix[ppp][1] = avg_angle_diff_01 + avg_angle_diff_02;
								angle_matrix[ppp][2] = avg_angle_diff_01 + avg_angle_diff_03;
								angle_matrix[ppp][3] = avg_angle_diff_01 + avg_angle_diff_04;

								while(angle_matrix[ppp][0] > PI) {
										angle_matrix[ppp][0] = angle_matrix[ppp][0] - TWO_PI;
								}
								while(angle_matrix[ppp][0] < (-PI) ) {
										angle_matrix[ppp][0] = angle_matrix[ppp][0] + TWO_PI;
								}
								
								while(angle_matrix[ppp][1] > PI) {
										angle_matrix[ppp][1] = angle_matrix[ppp][1] - TWO_PI;
								}
								while(angle_matrix[ppp][1] < (-PI) ) {
										angle_matrix[ppp][1] = angle_matrix[ppp][1] + TWO_PI;
								}
								
								while(angle_matrix[ppp][2] > PI) {
										angle_matrix[ppp][2] = angle_matrix[ppp][2] - TWO_PI;
								}
								while(angle_matrix[ppp][2] < (-PI) ) {
										angle_matrix[ppp][2] = angle_matrix[ppp][2] + TWO_PI;
								}
								
								while(angle_matrix[ppp][3] > PI) {
										angle_matrix[ppp][3] = angle_matrix[ppp][3] - TWO_PI;
								}
								while(angle_matrix[ppp][3] < (-PI) ) {
										angle_matrix[ppp][3] = angle_matrix[ppp][3] + TWO_PI;
								}
								
								data_analysis_index_start = data_analysis_index_start + _N_TBT;
								data_analysis_index_finish = data_analysis_index_finish + _N_TBT;
						}

			 			if (error_on_atan2 == 0){
							delta_IF_calc = avg_angle * _f_TBT / TWO_PI ;			 				
			 				for(sw_pos=0; sw_pos<N_SW_POS; sw_pos++)
								for(channel=0; channel<ALL_CH; channel++){	
									comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel] -= (float)angle_matrix[sw_pos][channel];

       								while(comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel] > PI) {
   										comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel] -= TWO_PI;
       								}
         								
       								while(comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel] < (-PI) ) {
       									comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel] += TWO_PI;
       								}
									DSC_Set_phase(current_pattern[sw_pos], sw_table[current_pattern[sw_pos] * ALL_CH + channel], comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel], delta_IF_calc);
								}
							comp_dbase [ATT_TABLE_OFFS + input_level].status |= PHASE_COMP_VALID;
						}	
						else{
								_LOG_ERR("'atan2' HAS BOTH ARGUMENTS ZERO. PROBLEM IN DSC's 'compensate_phase' FUNCTION !!!");
						}

					#if DEVEL
						phase_test_counter++;
						if (phase_test_counter == 5) {
							phase_test_counter = 0;
							syslog(LOG_INFO, "===================================================================================");							
							for (sw_pos=0; sw_pos<N_SW_POS; sw_pos++)
								for (channel=0; channel<ALL_CH; channel++){
									syslog(LOG_INFO, "angle_matrix[%d][%d]: %5.10f ", sw_pos, channel, comp_dbase [ATT_TABLE_OFFS + input_level].phase[sw_pos][channel]);
								}
							syslog(LOG_INFO, "===================================================================================");
						}
						
					#endif // DEVEL

						DSC_Apply_all();
				}
		}
		else 
				_LOG_ERR("cspi_read_ex: %s: %s", cspi_strerror(res), strerror(errno));
		
		return 0;
}

//**************************************************************************************************	
void inline calc_angle_diff(int * input_1, int * input_2, int * input_3, int * input_4, double * angle_out, double * angle_temp, long array_start, long array_finish)
//**************************************************************************************************	
{
		long kkk, nnn, ppp=0;
		nnn=0;
		ppp = array_start;
		
		for (kkk = array_start; kkk <= array_finish; kkk++) {
			
				if ( (input_1[ppp] == 0) && (input_2[ppp] == 0) ) {
						error_on_atan2 = 1;
				}
				else {
						angle_out[nnn] = (double)cordic_DSC_phase(input_1[ppp], input_2[ppp])* CORDIC_LONG_TO_FLOAT_NORM;
				}
				
				if ( (input_3[ppp] == 0) && (input_4[ppp] == 0) ) {
						error_on_atan2 = 1;
				}
				else {
						angle_temp[nnn] = (double) cordic_DSC_phase(input_3[ppp], input_4[ppp]) * CORDIC_LONG_TO_FLOAT_NORM;
				}
				nnn++;
				ppp = ppp + PHASE_INDEX_INCREMENT;
		}						
		correct_phase_to_absolute_value(angle_out, 0, (array_finish-array_start));
		correct_phase_to_absolute_value(angle_temp, 0, (array_finish-array_start));
		
		for (kkk = 0; kkk <= (array_finish-array_start); kkk++){
				angle_out[kkk] = angle_out[kkk] - angle_temp[kkk];
		}
	
		return;
}

//**************************************************************************************************	
void inline atan2_array(int * y_array, int * x_array, long array_start, long array_finish, double *atan2_output)
//**************************************************************************************************	
{
		long kkk, nnn, ppp;

		nnn = 0;
		ppp = array_start;		
		for (kkk = array_start; kkk <= array_finish; kkk++) {
				if ( (y_array[ppp]==0) && (x_array[ppp]==0) ){
						error_on_atan2 = 1;
				}
				else {
 						atan2_output[nnn] = (double) cordic_DSC_phase(x_array[ppp], y_array[ppp]) * CORDIC_LONG_TO_FLOAT_NORM;	
				}
				ppp = ppp + PHASE_INDEX_INCREMENT;
				nnn++;
		}						
}

//**************************************************************************************************	
void inline correct_phase_to_absolute_value(double *angle_inout, long array_start, long array_finish)
//**************************************************************************************************	
{
		int kkk;
		double add_phase = 0;
		for (kkk = array_start+1; kkk <= array_finish; kkk++) {
				angle_inout[kkk] = angle_inout[kkk] + add_phase;
				if ( fabs(angle_inout[kkk] - angle_inout[kkk-1]) > THREE_HALF_PI) {
						
						if( angle_inout[kkk] > angle_inout[kkk-1] ){
								add_phase = add_phase - TWO_PI;
								angle_inout[kkk] = angle_inout[kkk] - TWO_PI;
						}
						else{
								add_phase = add_phase + TWO_PI;
								angle_inout[kkk] = angle_inout[kkk] + TWO_PI;
						}
				}
		}						
}

//**************************************************************************************************	
double inline mean_value(double *data_input, long array_start, long array_finish)
//**************************************************************************************************	
{
		int kkk;
		double ret_val = 0;
		for (kkk = array_start; kkk <= array_finish; kkk++) {
				ret_val = ret_val + data_input[kkk];
		}
		ret_val = ret_val / (array_finish-array_start+1) ;
		return ret_val;						
}


//**************************************************************************************************	
int compensate_crosstalk()
//**************************************************************************************************	
{
	// TODO: add code for crosstalk compensation
	_LOG_DEBUG(">>>>>>>>>>>>>>>> compensating crosstalk <<<<<<<<<<<<<<<<<<<<<");
	return 0;
}

//**************************************************************************************************	
int compensate_gain()
//**************************************************************************************************	
{

int n_read = 0;
int sw_pos, sample, channel;


long long amplitude_sum;														// temporary result
int amplitude;																	// calculated single amplitude
int max_amplitude, min_amplitude;
int avg_samples;

int I=0, Q1=0, Q2=0, Q=0;														// I, samples for calc of Q and calculated Q
int res;
int amplitudes[ADC_READ_SIZE];													// calculated amplitudes
int peaks[ALL_CH] = {0,0,0,0};													// peak values for all four channels
float K_filt_AGC = 1;															// AGC filter constant
float power = 0;
float hysteresis = 0.50;														// default hysteresis
float delta_p = 0;																// power_difference


	if ((AGC_mode & AGC_mode_mask) == CSPI_AGC_MANUAL)							// if in manual mode
		return 0;																// finish here	
		
	_LOG_DEBUG(">>>>>>>>>>>>>>>> compensating gain (AGC) <<<<<<<<<<<<<<<<<<<<<");
	
	for(sw_pos = 0; sw_pos < N_SW_POS; sw_pos ++) {
//		_LOG_DEBUG(" position %d - acquiring %d atoms of ADCRB data...", current_pattern[sw_pos],  ADC_READ_SIZE );		
		
		if (DSC_switch == CSPI_SWITCH_AUTO)
			DSC_Set_ADCRB_trigger (current_pattern[sw_pos], ADCRB_DELAY, ADCRB_DSC_TRIG, NORMAL_T);	// set normal ADCRB trigger				
		else
			DSC_Set_ADCRB_trigger (current_pattern[sw_pos], ADCRB_DELAY, ADCRB_DSC_TRIG, FORCED_T);	// set forced ADCRB trigger
		
		ADCRB_wait_trigger (10); // TMP !!!!!!!!!!!!!!!!! 
	

		res = cspi_read(hcon_adc, ADC_buffer, ADC_READ_SIZE, &n_read);
		if ((res == CSPI_OK) && (n_read == ADC_READ_SIZE)) {					// only if required number of atoms was read
			_LOG_DEBUG("%d ADC samples acquired...", n_read);			
			for (channel = 0; channel < ALL_CH; channel ++) {					// through all channels
				_LOG_DEBUG("sw_pos: %d   channel: %d", sw_pos, channel);
				
				max_amplitude = 0;
				for (sample = 0; sample < ADC_READ_SIZE-1-AGC_skip_N; sample ++) {			// through all samples
					switch (channel) {
						case 0: I = ADC_buffer[sample].chA; Q1 = ADC_buffer[sample+AGC_skip_N].chA; break; Q2 = ADC_buffer[sample+AGC_skip_N+1].chA; break;
						case 1: I = ADC_buffer[sample].chB; Q1 = ADC_buffer[sample+AGC_skip_N].chB; break; Q2 = ADC_buffer[sample+AGC_skip_N+1].chB; break;
						case 2: I = ADC_buffer[sample].chC; Q1 = ADC_buffer[sample+AGC_skip_N].chC; break; Q2 = ADC_buffer[sample+AGC_skip_N+1].chC; break;
						case 3: I = ADC_buffer[sample].chD; Q1 = ADC_buffer[sample+AGC_skip_N].chD; break; Q2 = ADC_buffer[sample+AGC_skip_N+1].chD; break;
						default: I = 0; Q1 = 0; Q2 = 0;
					}
					
					Q = (delay_K1 * Q1 + delay_K2 * Q2) >> Q_SHIFT;				// Q from I
					amplitude = cordic_amp(I, Q);								// amplitude from I and Q - fast one
//					amplitude = (int)sqrt(I*I + Q*Q);							// old fashioned way of calculation - for test only
					amplitudes[sample] = amplitude;								// store calculated amplitude
						
					if (amplitude > max_amplitude)								// find peak value
						max_amplitude = amplitude;
				}
				
				min_amplitude = AMP_MIN_F * max_amplitude;						// calculate average top amplitude
				avg_samples = 0;
				amplitude_sum = 0;
				
				for (sample = 0; sample < ADC_READ_SIZE-1-AGC_skip_N; sample ++)// go through all samples
					if (amplitudes [sample] > min_amplitude) {					// process only peak values
						amplitude_sum += amplitudes[sample];
						avg_samples ++;
					}

				if(avg_samples)													// for sanity reason
					peaks[de_sw_table[current_pattern[sw_pos]* ALL_CH + channel]] += (int)(amplitude_sum /avg_samples);
			
				
/*				_LOG_DEBUG("peak_buff: %d   add:%d   sum: %d",sw_table[current_pattern[sw_pos]* ALL_CH + channel],
																(int)(amplitude_sum /avg_samples),
																peaks[de_sw_table[current_pattern[sw_pos]* ALL_CH + channel]] );*/
			}
		}
		DSC_Set_ADCRB_trigger (current_pattern[0],0, ADCRB_EXT_TRIG, NORMAL_T);	// return control to external trigger
		
	}

	// now we finally have all four averaged amplitudes, lets use them for something benefficial
	
	max_amplitude = 0;															// find highest amplitude
	for (channel = 0; channel < ALL_CH; channel ++){ 							// through all channels
		peaks[channel] = peaks[channel] / N_SW_POS / CG;						// average it through all switch positions in a sequence and 
																				// compensate CORDIC gain
		_LOG_DEBUG("channel: %d  peak: %d", channel, peaks[channel]);
		if (peaks[channel] > max_amplitude)
			max_amplitude = peaks[channel];
	}
	
	if (peak_filtered  > 0) {
		if (abs(max_amplitude - peak_filtered)/peak_filtered > (1-AMP_MIN_F)/2)	// huge difference;
			K_filt_AGC = K_F_AGC;
		else																	// small difference
			K_filt_AGC = K_S_AGC;
	}
	else																		// this almost can't happen but let's use fast filter
		K_filt_AGC = K_F_AGC;
	
	peak_filtered = max_amplitude * K_filt_AGC + (1-K_filt_AGC) * peak_filtered;// moving average filter
	
	power = 20 * log (peak_filtered / _ADCPEAK_0dBm)/log(10) +
			 + ATT_sum - _ATTNSUM_0dBm;											// calculated absolute input power	
	
	_LOG_DEBUG("max amplitude: %d   filtered peak: %.3f   K_filt: %.3f  power: %.2f", max_amplitude, peak_filtered, K_filt_AGC, power);

	hysteresis = ATT_table [input_level + ATT_TABLE_OFFS].hysteresis;			// read hysteresis for current level
	delta_p = power - (float)input_level;

// regulator logic	
	if (abs(delta_p) < SLOW_P_CHG_LIMIT) {										// small difference --> slow change
		if (delta_p < ( - 0.5 - hysteresis)) {									// outside area of current attenuator and hysteresis?
			input_level --;
			DSC_apply_new_settings();
		}
		
		if (delta_p >= (+ 0.5 + hysteresis)) {									// outside area of current attenuator and hysteresis?
			input_level ++;
			DSC_apply_new_settings();
		}			
	}
	else {																		// big difference -->
		input_level += (int)(delta_p * FAST_ATT_CHANGE);						// apply correction fast, but not at once
		DSC_apply_new_settings();												// also input_level range check is performed here
	}
	
	_LOG_DEBUG("AGC sets input level: %d dBm", input_level);

	return 0;
}

//**************************************************************************************************	
void DSC_apply_new_settings(int call_from) {
//**************************************************************************************************	
// this function is executed when a CSPI request a change either in DSC mode / switches or 
// new input signal level is reported.

// DSC algorithms don't use this function as it sets everything regarding the DSC, not only for example
// amplitude or phase compensation
unsigned char atts[ALL_CH * 2];

	if (input_level > highest_att_entry)											// protection - set to a known level
		input_level = highest_att_entry;

	if (input_level < lowest_att_entry)												// protection - set to a known level
		input_level = lowest_att_entry;

	// only if change in level, DSC mode or switch than apply corresponding coefficients to the FPGA
	if ((input_level != old_input_level) || (DSC_mode != old_DSC_mode) || (DSC_switch != old_DSC_switch)) {
		
		DSC_Set_sw_pattern (current_pattern, N_SW_POS);							// turn on switching
		DSC_Set_TBT_marker (current_pattern[0], _TBT_M_DELAY);					// marker to first position in array		

		// copy attenuator values from table
		for (int i = 0; i < ALL_CH; i++) {
			atts[i*2] = (unsigned char) ATT_table [input_level + ATT_TABLE_OFFS].att1;
			atts[i*2+1] = (unsigned char) ATT_table [input_level + ATT_TABLE_OFFS].att2;
		}
		DSC_set_att(atts);
		ATT_sum = ATT_table[input_level + ATT_TABLE_OFFS].att1 + ATT_table [input_level + ATT_TABLE_OFFS].att2;
		
		
		// copy coefficients
		for (unsigned char sw_pos = 0; sw_pos < N_SW_POS; sw_pos++)
			for (unsigned char channel = 0; channel < ALL_CH; channel++) {
				
				// if unity values required
				if (DSC_mode == CSPI_DSC_UNITY) {
					DSC_Set_gain (current_pattern[sw_pos], sw_table[current_pattern[sw_pos] * ALL_CH + channel], 1.0000);
					DSC_Set_phase (current_pattern[sw_pos], sw_table[current_pattern[sw_pos] * ALL_CH + channel], 0.0000, 0);
				}
				else {
					// otherwise use what is currently in the dbase regardles switces are still or rotating
					// in both cases write to the table that will be copied to the FPGA
					// here is also done tranlation into absolute positions in coresponding RF chains - LUTs are addressed					
					DSC_Set_gain (pos_array_rotating[sw_pos], sw_table[pos_array_rotating[sw_pos] * ALL_CH + channel],
									comp_dbase[ATT_TABLE_OFFS + input_level].ampl[sw_pos][channel]);
					DSC_Set_phase (pos_array_rotating[sw_pos], sw_table[pos_array_rotating[sw_pos] * ALL_CH + channel], 
									comp_dbase[ATT_TABLE_OFFS + input_level].phase[sw_pos][channel], 0);
					}					
				}				

		DSC_Apply_all();
	}
	
	old_input_level = input_level;													// after checking renew history values of settings
	old_DSC_mode = DSC_mode;
	old_DSC_switch = DSC_switch;
}


//**************************************************************************************************	
int AGC_read_table() {
//**************************************************************************************************		

FILE *fp = NULL;
char * line = NULL;

int level;
unsigned int att1, att2;
float hysteresis;
unsigned int time;
size_t len = 0;
unsigned int line_no = 0;
int rc = 0;

	fp = fopen(GAIN_FILENAME, "r");

	if (fp == NULL)
	 	return -1;
		
	_LOG_DEBUG("ATT table file content:");
	_LOG_DEBUG("P A1 A2 h   t");

	for (level = 0; level < (MAX_INP_POWER - MIN_INP_POWER+1); level++) {
		ATT_table [level].att1 = 0xFF;													// mark all entries as undefined
		ATT_table [level].att2 = 0xFF;		
	}
	
	ssize_t nread;		
 	while ((nread = getline( &line, &len, fp )) != -1 ) {
	 
		line_no ++;																		// current line
	 		
		const char *p = line;
	 	while ( isspace(*p) ) { ++p; }
	 	if (*p == '\0' || *p == '#') continue;
	 
		if (sscanf (p, "%d %d %d %f %d", &level, &att1, &att2, &hysteresis, &time) == 5) {// properly read parameters
    	
			_LOG_DEBUG("%d %d %d %.2f %d", level, att1, att2, hysteresis, time);
    	
			if ((att1 <0) ||(att1 > MAX_ATT) || (att2 <0) || (att2 > MAX_ATT)) {
				syslog( LOG_ERR, "%s: attenuator value in line %d outside limits, aborting file read...", GAIN_FILENAME, line_no);// report the error
				rc = -1;
				break;
			}
    	
			if ((level < MIN_INP_POWER) || (level > MAX_INP_POWER)) {
				syslog(LOG_ERR, "%s: input level in line %d outside limits, aborting file read...", GAIN_FILENAME, line_no);// report the error
				rc = -1;
				break;
			}
    	
    		if ((hysteresis <0) || (hysteresis > MAX_HISTER)) {
				syslog(LOG_ERR, "%s: hysteresis in line %d outside limits, aborting file read...", GAIN_FILENAME, line_no);// report the error
				rc = -1;
				break;
			}
    	
			ATT_table [level + ATT_TABLE_OFFS].att1 = att1;							// everything OK so far, store the attenuator values
			ATT_table [level + ATT_TABLE_OFFS].att2 = att2;
			ATT_table [level + ATT_TABLE_OFFS].hysteresis = hysteresis;				// store the hysteresis
    	
			if (level > highest_att_entry)											// store highest and lowest value
				highest_att_entry = level;
			
			if (level < lowest_att_entry)
				lowest_att_entry = level;
				
		}
		else {																		// number of parameters not OK
			syslog(LOG_ERR, "%s: error in line %d, aborting file read...", GAIN_FILENAME, line_no);// this is clearly an error
			rc = -1;
			break;
		}
 	}

	for (level = lowest_att_entry; level <=highest_att_entry ; level++)				// check if there are all entries present
		if ((ATT_table [level + ATT_TABLE_OFFS].att1 > MAX_ATT) || (ATT_table [level + ATT_TABLE_OFFS].att2 > MAX_ATT)) {
			syslog(LOG_ERR,"in gain.conf missing entry for power level %d dBm...",level);// this is clearly an error		
			rc = -1;
		}

	 		
	if ( line ) free( line );
	fclose(fp);
	return rc;
	
}


//**************************************************************************************************
int read_comp_coeff (unsigned char * filename) {
//**************************************************************************************************	
FILE * fd = NULL;
int	ret_val = 1;

	fd = fopen (filename, "r");
	if (fd) {
		if (fread (&comp_dbase, sizeof (comp_dbase), 1, fd) == 1) {
			ret_val = 0;
			_LOG_DEBUG ("%s successfully read", filename);
		}
		fclose (fd);
	}
	return (ret_val);
}


//**************************************************************************************************
int write_comp_coeff (unsigned char * filename) {
//**************************************************************************************************	
FILE * fd = NULL;
int	ret_val = -1;
	
	fd = fopen (filename, "wb");

	if (fd) {
		if (fwrite (&comp_dbase, sizeof (comp_dbase), 1, fd) == 1) {
			ret_val = 0;
			_LOG_INFO ("%s successfully written", filename);
		}
		fclose (fd);
		return (ret_val);		
	}
	else
		return (errno);
}
//**************************************************************************************************



