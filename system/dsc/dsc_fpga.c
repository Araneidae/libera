// $Id: dsc_fpga.c,v 1.16 2006/12/04 10:01:52 primoz Exp $

//! \file dsc_fpga.c
//! Implements DSC-FPGA interface.

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

#include "dsc_fpga.h"
#include "debug.h"


// switch positions in one sequence
#define N_SW_POS		8
#define DSC_READ_SIZE 	_N_TBT * N_SW_POS * 2
#define N_ACQ_AVG		4


dsc_struct storage_struct, storage_struct_old;			// main structure where all parameters are stored and its copy that was last written to the FPGA
int first_write;										// first write to the FPGA
int double_buffer;	
int PHGN_UNITY_NORM = PHGN_UNITY;						// normalized multiplier range (accelerator specific)



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//						PRIVATE FUNCTIONS
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

//**************************************************************************************************

// header files needed to compile /dev/mem access
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>	
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


int mem_fd = -1;
void *map_base = NULL;


/** /dev/mem access initialization. 
 *
 * On success, returns 0.
 * On failure, returns -1.
 */
//************************************************************************************************** 
int fpga_rw_init(unsigned long base_addr) 
//**************************************************************************************************
{
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) return -1;

    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base_addr);
    if(map_base == MAP_FAILED) return -1;

    return 0;
}

/** /dev/mem access cleanup. 
 *
 * On success, returns 0.
 * On failure, returns -1.
 */

//**************************************************************************************************
int fpga_rw_cleanup()
//**************************************************************************************************
{
    if( munmap(map_base, MAP_SIZE) == -1 ) return -1;

    close(mem_fd);
    
    return 0;
}

/** Writes a 32-bit value to fpga. 
 *
 * Writes the vaule @param val to address offset @param off.
 */
//**************************************************************************************************
void fpga_write(unsigned long off, unsigned long val)
//**************************************************************************************************
{
    *( (unsigned long *) (map_base + (off & MAP_MASK)) ) = val;			

#ifdef __DEVEL
    _LOG_DEBUG("W(0x%08lx) = 0x%08lx\n", (unsigned long)(off + FPGA_BASE_ADDR), val);			
#endif
}

/** Reads a 32-bit value from fpga. 
 *
 * Returns the vaule, read from address offset @param off.
 */
//************************************************************************************************** 
unsigned long fpga_read(unsigned long off)
//**************************************************************************************************
{
    unsigned long val = *( (unsigned long *) (map_base + (off & MAP_MASK)) );

#ifdef __DEVEL
    _LOG_DEBUG("R(0x%08lx) = 0x%08lx\n", (unsigned long)(off + FPGA_BASE_ADDR), val);			
#endif

    return val;
}

//**************************************************************************************************
int FPGA_write_sw_pattern (void) {
//**************************************************************************************************

#define DB_SW_MEM_SPACE		(DSC_PATTERN_GEN_TOP-DSC_PATTERN_GEN_BASE+1)/2

	off_t target=0;

	unsigned int storage_addr = 0;
	unsigned int write_val = 0;
	unsigned int curr_pos;
	unsigned int rpt_no;

	
//---------------------------------------------------------------------------------------------------
// pattern size >= N_POS_IN_INT	
//---------------------------------------------------------------------------------------------------
	if (storage_struct.no_of_positions >= N_POS_IN_INT) {
    	while (storage_addr < (DB_SW_MEM_SPACE / 4)) {								// store the pattern generator content into the FPGA
			for (curr_pos = 0; curr_pos < storage_struct.no_of_positions; curr_pos ++) {
				if (curr_pos % N_POS_IN_INT == 0)
					write_val = 0;													// empty new int
				write_val |= (storage_struct.positions[curr_pos]) << ((curr_pos % N_POS_IN_INT) * 32 / N_POS_IN_INT);	// add new position to integer
				if (curr_pos % N_POS_IN_INT == (N_POS_IN_INT-1)) {					// one int is full, store it!
					// write to the FPGA address
					target = DSC_PATTERN_GEN_BASE + ((double_buffer+1)%2) * DB_SW_MEM_SPACE +
								storage_addr * 4 + (curr_pos / N_POS_IN_INT) * 4;
					fpga_write(target, write_val);
				}
			}
   			storage_addr += storage_struct.no_of_positions/N_POS_IN_INT;			// next repetition of pattern
		}
    }	

//---------------------------------------------------------------------------------------------------
// pattern size < N_POS_IN_INT		
//---------------------------------------------------------------------------------------------------
	else {
		while (storage_addr < (DB_SW_MEM_SPACE / 4)) {								// store the pattern generator content into the FPGA
			write_val=0;
			for (rpt_no = 0; rpt_no < N_POS_IN_INT/storage_struct.no_of_positions; rpt_no ++)
				for (curr_pos = 0; curr_pos < storage_struct.no_of_positions; curr_pos ++)
					write_val |= ( storage_struct.positions[curr_pos]) << (32/N_POS_IN_INT * (curr_pos + rpt_no * storage_struct.no_of_positions));
					
			target = DSC_PATTERN_GEN_BASE + ((double_buffer + 1) % 2) * DB_SW_MEM_SPACE + storage_addr * 4;					
			fpga_write(target, write_val);

			storage_addr ++;
		}
	}
	
	return 0;
}


//**************************************************************************************************
int FPGA_write_phase_gain (void) {
//**************************************************************************************************

#define DB_PHGN_MEM_SPACE		(DSC_PHASE_GAIN_TOP-DSC_PHASE_GAIN_BASE+1)/2

	off_t target=0;

	unsigned int sw_position;
	unsigned int channel;
	unsigned int storage_addr = 0;	
	int K1;
	int K2;
	double temp_K1;
	double temp_K2;	
	double temp_angle;

	for (channel = 0; channel < MAX_CHANNELS; channel ++)
		for(sw_position = 0; sw_position < MAX_SW_POSITIONS; sw_position ++) {
			
			// additional angle introduced by the detune
			temp_angle =  2 * PI * ((double) storage_struct.freq_IF + storage_struct.delta_IF)  / (storage_struct.freq_samp);

			// direct path
			temp_K1 = -sin( (double)(storage_struct.phase[sw_position][channel]) );
			temp_K1 = temp_K1/tan(temp_angle);
			temp_K1 = temp_K1 + cos( (double)(storage_struct.phase[sw_position][channel]) );
			temp_K1 = temp_K1 * (double)(storage_struct.gain[sw_position][channel]);
			// transform to multiplier range			
			temp_K1 *= PHGN_UNITY_NORM;
			K1 = (int) round(temp_K1);
			
			// delayed path
			temp_K2 = sin( (double)(storage_struct.phase[sw_position][channel]) );
			temp_K2 = temp_K2/sin(temp_angle);
			temp_K2 = temp_K2 * (double)(storage_struct.gain[sw_position][channel]);
			// transform to multiplier range
			temp_K2 *= PHGN_UNITY_NORM;			
			K2 = (int) round(temp_K2);
			
#ifdef DEVEL
			if ( (channel == 0) && (sw_position==0) ) {
					syslog(LOG_INFO, "!!!!!!!!!!---------!!!!!!!!!!!!");
					syslog(LOG_INFO, "FILE 'dsc_fpga.c'");
					syslog(LOG_INFO, "Matrix element[0][0]");
					syslog(LOG_INFO, "phase correction angle: %5.10f ", storage_struct.phase[sw_position][channel]);
					syslog(LOG_INFO, "NCO frequency: %15.10f ", storage_struct.freq_IF);
					syslog(LOG_INFO, "VCO frequency: %25.10f ", storage_struct.freq_samp);
					syslog(LOG_INFO, "NCO correction: %15.10f ", storage_struct.delta_IF);
					syslog(LOG_INFO, "Gain correction: %15.10f ", storage_struct.gain[sw_position][channel]);
					syslog(LOG_INFO, "Phase Normalization: %d ", PHGN_UNITY);
					syslog(LOG_INFO, "K1_DSC: %d ", K1);
					syslog(LOG_INFO, "K2_DSC: %d ", K2);
					syslog(LOG_INFO, "!!!!!!!!!!---------!!!!!!!!!!!!");
			}
#endif			

			storage_addr = 128 * channel + sw_position * 8;

			// write to the FPGA address
			target = DSC_PHASE_GAIN_BASE + ((double_buffer+1)%2) * DB_PHGN_MEM_SPACE + storage_addr;
			fpga_write(target, K1);
			
			target = DSC_PHASE_GAIN_BASE + ((double_buffer+1)%2) * DB_PHGN_MEM_SPACE + storage_addr + 4;
			fpga_write(target, K2);
		
		}
		
	return 0;
}

//**************************************************************************************************
int FPGA_write_delays(void) {
//**************************************************************************************************

	unsigned int write_val;

	write_val = storage_struct.analog_2_digital_delay << 16;
	fpga_write(DSC_DELAY_CR, write_val);	

	return 0;
}

//**************************************************************************************************
int FPGA_write_xtalk (void) {
//**************************************************************************************************

#define DB_XTALK_MEM_SPACE		(DSC_XTALK_TOP-DSC_XTALK_BASE+1)/2

	off_t target=0;

	unsigned int position;
	unsigned int target_addr;
	unsigned int frm_ch;
	unsigned int to_ch;
	int calc_coeff;

	// this function first creates XTALK block image
	for (position = 0; position < MAX_SW_POSITIONS; position ++)
		for (frm_ch = 0; frm_ch < MAX_CHANNELS; frm_ch ++)
			for (to_ch = 0; to_ch < MAX_CHANNELS; to_ch ++) {
				target_addr = to_ch * 64 + (frm_ch / 2) * 32 + position * 2 + frm_ch % 2;
				calc_coeff = (int)(storage_struct.xtalk[position][frm_ch][to_ch] * K_XTALK_UNITY);

				target = DSC_XTALK_BASE + ((double_buffer+1)%2) * DB_XTALK_MEM_SPACE + target_addr * 4;
				fpga_write(target, calc_coeff);	
				
			}
		
	return 0;
}


//**************************************************************************************************
int FPGA_write_ATTs(void) {
//**************************************************************************************************

	off_t target;	
	unsigned int write_val;

	target = DSC_ATT_L + ((double_buffer+1)%2) * ATT_BANK_SIZE;
	write_val = ((storage_struct.attenuators[7] & 0x1f) << 24) | 		// D2
				((storage_struct.attenuators[6] & 0x1f) << 16) | 		// D1
				((storage_struct.attenuators[5] & 0x1f) << 8) |			// C2
				((storage_struct.attenuators[4] & 0x1f));				// C1
				
	fpga_write(target, write_val);

	
	target = DSC_ATT_H + ((double_buffer+1)%2) * ATT_BANK_SIZE;
	write_val = ((storage_struct.attenuators[3] & 0x1f) << 24) | 		// B2
				((storage_struct.attenuators[2] & 0x1f) << 16) | 		// B1
				((storage_struct.attenuators[1] & 0x1f) << 8) |			// A2
				((storage_struct.attenuators[0] & 0x1f));				// A1

	fpga_write(target, write_val);

	return 0;
}


//**************************************************************************************************
void FPGA_toggle_double_buffer(void) {
//**************************************************************************************************
	
	double_buffer=(double_buffer+1)%2;
	fpga_write(DSC_DOUBLE_BUFF_CR, double_buffer);
}


//**************************************************************************************************
int DSC_check_phase_gain (unsigned int sw_position, unsigned int channel) {
//**************************************************************************************************
int K1, K2;

	// delayed path
	K1 = (-sin(storage_struct.phase[sw_position][channel]) /
		tan(2*PI*storage_struct.freq_IF/storage_struct.freq_samp) +
		cos(storage_struct.phase[sw_position][channel])) * storage_struct.gain[sw_position][channel] * PHGN_UNITY_NORM;

	// delayed path
	K2 = (sin(storage_struct.phase[sw_position][channel])/
		sin(2*PI*storage_struct.freq_IF/storage_struct.freq_samp)) * storage_struct.gain[sw_position][channel] * PHGN_UNITY_NORM;
		
	if ((abs(K1) > MAX_K1_K2) || (abs(K2) > MAX_K1_K2))
		return PHGN_COEF_OUT_OF_RANGE;
	else
		return 0;
}



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//						PUBLIC FUNCTIONS
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


 //**************************************************************************************************
 // Apply all settings to the FPGA
 int DSC_Apply_all (void) {
 //**************************************************************************************************
 	
 int return_val;
 	
 	// this prevents too much writting to the FPGA
 	// writing does not execute if the content to be written is the same as previous
 	if (first_write || (memcmp(&storage_struct_old, &storage_struct, sizeof (dsc_struct)) != 0)) {	// if it needs to be written to the FPGA
 		//_LOG_DEBUG ("first write %d\n", first_write);
 		//_LOG_DEBUG ("memcmp %d\n", memcmp(&storage_struct_old, &storage_struct, sizeof (dsc_struct)));		
 		
 		_LOG_DEBUG("\nwritting DSC switching pattern into FPGA...\n");
 		if ((return_val = FPGA_write_sw_pattern ()) != 0) {
 			_LOG_DEBUG("Error code: %d\n", return_val);
 			return (return_val);
 		}
 		
 		_LOG_DEBUG("\nwritting DSC phase & gain into FPGA...\n");
 		if ((return_val = FPGA_write_phase_gain ()) != 0) {
 			_LOG_DEBUG("Error code: %d\n", return_val);
 			return (return_val);
 		}	
 	
 		_LOG_DEBUG("\nwritting DSC A2D & TBT delays into FPGA...\n");
 		if ((return_val = FPGA_write_delays ()) != 0) {
 			_LOG_DEBUG("Error code: %d\n", return_val);
 			return (return_val);
 		}
 	
 		_LOG_DEBUG("\nwritting DSC XTALK matrices into FPGA...\n");
 		if ((return_val = FPGA_write_xtalk ()) != 0) {
 			_LOG_DEBUG("Error code: %d\n", return_val);
 			return (return_val);
 		}
 	
 		_LOG_DEBUG("\nwritting ATTs into FPGA...\n");		
 		if ((return_val = FPGA_write_ATTs ()) != 0) {
 			_LOG_DEBUG("Error code: %d\n", return_val);
 			return (return_val);
 		}
 	
 		memcpy (&storage_struct_old, &storage_struct, sizeof (dsc_struct));
 		
 		_LOG_DEBUG("\ntoggling double buffer bit...\n");		
 		FPGA_toggle_double_buffer ();
 		
 		first_write = 0;
 	}
 	
 	return 0;
 }


//**************************************************************************************************
// Switching pattern generator settings
// * positions is a pointer to array of consequtive possitions
// * target_struct is a pointer to complete DSC settings
// pos_no must be 2^n (1, 2, 4, 8, 16,...2048);
// function returns number of positions copied, negative number is error code

int DSC_Set_sw_pattern (unsigned int * positions, unsigned int pos_no) {
//**************************************************************************************************	
	unsigned int pos_base = 0x00000001;	
	int pos_counter;

	while (pos_no / pos_base) {											// determine/truncate number of positions to 2^n
		storage_struct.no_of_positions = pos_base;						// actual positions used			
		pos_base <<= 1;													// enlarge divider
	}

	if (storage_struct.no_of_positions <= MAX_SW_PATTERN){
		for (pos_counter = 0; pos_counter < storage_struct.no_of_positions; pos_counter ++)	// copy positions
			if (positions[pos_counter] < MAX_SW_POSITIONS)
				storage_struct.positions[pos_counter] = positions[pos_counter];
			else
				return (ERR_INVALID_POSITION);
		return(storage_struct.no_of_positions);
	}
	else
		return (ERR_TOO_MANY_POSITIONS);
};



//**************************************************************************************************
int DSC_Set_ana2dig_delay (unsigned int ana2dig_delay){
//**************************************************************************************************

	if (ana2dig_delay > MAX_ANA2DIG_DELAY)
		return (ERR_ANA2DIG_DELAY_TOO_LONG);
	else {
		storage_struct.analog_2_digital_delay = ana2dig_delay;
		return 0;
	}
}


//**************************************************************************************************
// Gain compensation settings
// gain settable from -2 to +2
int DSC_Set_gain (unsigned int sw_position, unsigned int channel, float gain) {
//**************************************************************************************************	

	storage_struct.gain[sw_position][channel] = gain;
	return(DSC_check_phase_gain(sw_position, channel));
}


//**************************************************************************************************
// Phase compensation settings
// delay settable from -Pi to +Pi ???
int DSC_Set_phase (unsigned int sw_position, unsigned int channel, float delay, double delta_IF) {
//**************************************************************************************************	
		
	storage_struct.phase[sw_position][channel] = delay;
	storage_struct.delta_IF = delta_IF;
	return(DSC_check_phase_gain(sw_position, channel));
}
	

//**************************************************************************************************
int DSC_Set_xtalk (unsigned int sw_position, unsigned int frm_channel, unsigned int to_channel, float value) {
//**************************************************************************************************	

	storage_struct.xtalk[sw_position][frm_channel][to_channel] = value;
	if (abs (value * K_XTALK_UNITY) > MAX_XTALK)
		return XTALK_COEF_OUT_OF_RANGE;
	else
		return 0;
}

//**************************************************************************************************
// ADC Rate buffer trigger settings
// <delay> in ADC clock samples
// <trig_src> = 0 -> external trigger
// <trig_src> = 1 -> DSC
int DSC_Set_ADCRB_trigger (unsigned int position, unsigned int delay, unsigned int trig_src, unsigned int forced) {
//**************************************************************************************************	

	unsigned int write_val;
	
	if (position >= MAX_SW_POSITIONS)
		return SW_POSITION_OUT_OF_RANGE;	

	if (delay > MAX_ADCRB_TRIG_DELAY)
		return ADCRB_TRIG_DELAY_OUT_OF_RANGE;
		
	if (delay < MIN_ADCRB_TRIG_DELAY) 
		delay = MIN_ADCRB_TRIG_DELAY;
	
	if (trig_src == ADCRB_EXT_TRIG)													// can't be forced if external
		forced = 0;
	
	if (trig_src)
		write_val = 0x00000001;														// DSC selected
	else
		write_val = 0x00000000;														// ext. trig selected

	fpga_write(ADC_SCOPE_CTRL, write_val);
			 
		
	write_val = ((position & 0x0000000f) << 16) | delay;							// set switch position & delay
	if (trig_src)																	// also arm if internal source is selected
		write_val |= 80000000;
	
	fpga_write(DSC_ADCRB_TRIGGER_CR, write_val);
	_LOG_DEBUG("1st write to trigger - pos %d", position);


	if (forced) {
		fpga_write(DSC_ADCRB_TRIGGER_CR, write_val);
		_LOG_DEBUG("2nd write to trigger");	
		usleep (3);
		if(fpga_read(DSC_ADCRB_TRIGGER_CR) & 0x80000000)							// check if accidentaly still armed
			fpga_write(DSC_ADCRB_TRIGGER_CR, write_val);							// if so, another write to force the triger
	}
		
	_LOG_DEBUG("\nADCRB DSC trigger set & armed(!)...\n");

	return 0;
	
}

//**************************************************************************************************
// waits for ADCRB trigger to happen and sleeps 10ms in-between
void ADCRB_wait_trigger (unsigned int maxt_x10ms) {
//**************************************************************************************************	
	while ((fpga_read(DSC_ADCRB_TRIGGER_CR) & 0x80000000) && (maxt_x10ms > 0)) {	
		usleep(10000);																// sleep 10ms and wait for the trigger to happen
		maxt_x10ms--;
	}
}

//**************************************************************************************************
// TMT marker settings
// <delay> in ADC clock samples [0 - 65535]
int DSC_Set_TBT_marker (unsigned int position, unsigned int delay) {
//**************************************************************************************************	

	unsigned int write_val;
	
	if (position >= MAX_SW_POSITIONS)
		return SW_POSITION_OUT_OF_RANGE;	

	if (delay > MAX_TBT_MARKER_DELAY)
		return TBT_MARKER_OUT_OF_RANGE;
		
	else {
		write_val = ((position & 0x0000000f) << 16) | (delay & 0xFFFF);
		fpga_write(DSC_HIST_BUFF_MARK_CR, write_val);				 

#ifdef __DSC_DEBUG		
		_LOG_DEBUG("\nTBT marker set...\n");
#endif //__DSC_DEBUG		
		
		return 0;
	}
}


//**************************************************************************************************
int DSC_set_att(unsigned char * attenuators) {
//**************************************************************************************************		
int i;
	for (i=0; i<MAX_ATTS; i++)
		storage_struct.attenuators[i] = (unsigned int) attenuators[i];
	return 0;
}


//**************************************************************************************************
//int DSC_init(float freq_samp, float freq_IF, int channel_order){
int DSC_init(double freq_samp, double freq_IF, int channel_order){	
//**************************************************************************************************

	unsigned int positions_ABCD[]={3};
	unsigned int positions_ABDC[]={7};
	unsigned int pos_no=1;
	unsigned int sw_position;
	unsigned int channel;
	unsigned int to;
	first_write = 1;												// needs to be written to the FPGA

	_LOG_DEBUG("configuring DSC initial settings...\n");
	_LOG_INFO ("fs=%f   fIF=%f\n", freq_samp, freq_IF);

	if (channel_order)
		DSC_Set_sw_pattern (positions_ABDC,  pos_no);				// initialy single position (3=pass through)
	else
		DSC_Set_sw_pattern (positions_ABCD,  pos_no);				// initialy single position (7=pass through)	
	
	storage_struct.analog_2_digital_delay = 0;						// initialy no delay between analog & digital
	storage_struct.freq_samp = freq_samp;							// store sampling frequency
	storage_struct.freq_IF=freq_IF;									// store IF frequency
	double_buffer=1;												// initialy FPGA is using higher space and 
																	// user is writting to lower space
	
	// set gain to unity and phase to 0
	for (sw_position=0; sw_position < MAX_SW_POSITIONS; sw_position ++)
		for (channel=0; channel < MAX_CHANNELS; channel ++) {
			DSC_Set_gain(sw_position,channel,1);
			DSC_Set_phase(sw_position,channel,0,0);
		}

	// clear entire xtalk matrices
	for (sw_position=0; sw_position < MAX_SW_POSITIONS; sw_position ++)
		for (channel=0; channel < MAX_CHANNELS; channel ++)
			for (to=0; to < MAX_CHANNELS; to ++)
				storage_struct.xtalk[sw_position][channel][to]=0;

	// calculate normalized unity multiplier value
	PHGN_UNITY_NORM = PHGN_UNITY * sin (2 * PI * storage_struct.freq_IF / storage_struct.freq_samp);
		
	if (channel_order == 0) {
#ifdef __DSC_DEBUG
		_LOG_DEBUG("input channel order ABCD\n");
#endif //__DSC_DEBUG

		// set xtalk matrices for de-switching only (ABCD input order)
		storage_struct.xtalk[0][CH_D][CH_A]=1;							// position 0
		storage_struct.xtalk[0][CH_C][CH_B]=1;
		storage_struct.xtalk[0][CH_B][CH_C]=1;
		storage_struct.xtalk[0][CH_A][CH_D]=1;
    	
    	
		storage_struct.xtalk[1][CH_D][CH_A]=1;							// position 1
		storage_struct.xtalk[1][CH_B][CH_B]=1;
		storage_struct.xtalk[1][CH_C][CH_C]=1;
		storage_struct.xtalk[1][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[2][CH_A][CH_A]=1;							// position 2
		storage_struct.xtalk[2][CH_C][CH_B]=1;
		storage_struct.xtalk[2][CH_B][CH_C]=1;
		storage_struct.xtalk[2][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[3][CH_A][CH_A]=1;							// position 3
		storage_struct.xtalk[3][CH_B][CH_B]=1;
		storage_struct.xtalk[3][CH_C][CH_C]=1;
		storage_struct.xtalk[3][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[4][CH_D][CH_A]=1;							// position 4
		storage_struct.xtalk[4][CH_C][CH_B]=1;
		storage_struct.xtalk[4][CH_A][CH_C]=1;
		storage_struct.xtalk[4][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[5][CH_D][CH_A]=1;							// position 5
		storage_struct.xtalk[5][CH_B][CH_B]=1;
		storage_struct.xtalk[5][CH_A][CH_C]=1;
		storage_struct.xtalk[5][CH_C][CH_D]=1;
    	
    	
		storage_struct.xtalk[6][CH_A][CH_A]=1;							// position 6
		storage_struct.xtalk[6][CH_C][CH_B]=1;
		storage_struct.xtalk[6][CH_D][CH_C]=1;
		storage_struct.xtalk[6][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[7][CH_A][CH_A]=1;							// position 7
		storage_struct.xtalk[7][CH_B][CH_B]=1;
		storage_struct.xtalk[7][CH_D][CH_C]=1;
		storage_struct.xtalk[7][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[8][CH_C][CH_A]=1;							// position 8
		storage_struct.xtalk[8][CH_D][CH_B]=1;
		storage_struct.xtalk[8][CH_B][CH_C]=1;
		storage_struct.xtalk[8][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[9][CH_B][CH_A]=1;							// position 9
		storage_struct.xtalk[9][CH_D][CH_B]=1;
		storage_struct.xtalk[9][CH_C][CH_C]=1;
		storage_struct.xtalk[9][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[10][CH_C][CH_A]=1;							// position 10
		storage_struct.xtalk[10][CH_A][CH_B]=1;
		storage_struct.xtalk[10][CH_B][CH_C]=1;
		storage_struct.xtalk[10][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[11][CH_B][CH_A]=1;							// position 11
		storage_struct.xtalk[11][CH_A][CH_B]=1;
		storage_struct.xtalk[11][CH_C][CH_C]=1;
		storage_struct.xtalk[11][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[12][CH_C][CH_A]=1;							// position 12
		storage_struct.xtalk[12][CH_D][CH_B]=1;
		storage_struct.xtalk[12][CH_A][CH_C]=1;
		storage_struct.xtalk[12][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[13][CH_B][CH_A]=1;							// position 13
		storage_struct.xtalk[13][CH_D][CH_B]=1;
		storage_struct.xtalk[13][CH_A][CH_C]=1;
		storage_struct.xtalk[13][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[14][CH_C][CH_A]=1;							// position 14
		storage_struct.xtalk[14][CH_A][CH_B]=1;
		storage_struct.xtalk[14][CH_D][CH_C]=1;
		storage_struct.xtalk[14][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[15][CH_B][CH_A]=1;							// position 15
		storage_struct.xtalk[15][CH_A][CH_B]=1;
		storage_struct.xtalk[15][CH_D][CH_C]=1;
		storage_struct.xtalk[15][CH_C][CH_D]=1;
	}

	else {
#ifdef __DSC_DEBUG
		_LOG_DEBUG("input channel order ABDC\n");
#endif //__DSC_DEBUG		
		// set xtalk matrices for de-switching only (ABDC input order)
		storage_struct.xtalk[0][CH_D][CH_A]=1;							// position 0
		storage_struct.xtalk[0][CH_C][CH_B]=1;
		storage_struct.xtalk[0][CH_A][CH_C]=1;
		storage_struct.xtalk[0][CH_B][CH_D]=1;
    	
    	
		storage_struct.xtalk[1][CH_D][CH_A]=1;							// position 1
		storage_struct.xtalk[1][CH_B][CH_B]=1;
		storage_struct.xtalk[1][CH_A][CH_C]=1;
		storage_struct.xtalk[1][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[2][CH_A][CH_A]=1;							// position 2
		storage_struct.xtalk[2][CH_C][CH_B]=1;
		storage_struct.xtalk[2][CH_D][CH_C]=1;
		storage_struct.xtalk[2][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[3][CH_A][CH_A]=1;							// position 3
		storage_struct.xtalk[3][CH_B][CH_B]=1;
		storage_struct.xtalk[3][CH_D][CH_C]=1;
		storage_struct.xtalk[3][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[4][CH_D][CH_A]=1;							// position 4
		storage_struct.xtalk[4][CH_C][CH_B]=1;
		storage_struct.xtalk[4][CH_B][CH_C]=1;
		storage_struct.xtalk[4][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[5][CH_D][CH_A]=1;							// position 5
		storage_struct.xtalk[5][CH_B][CH_B]=1;
		storage_struct.xtalk[5][CH_C][CH_C]=1;
		storage_struct.xtalk[5][CH_A][CH_D]=1;
    	
    	
		storage_struct.xtalk[6][CH_A][CH_A]=1;							// position 6
		storage_struct.xtalk[6][CH_C][CH_B]=1;
		storage_struct.xtalk[6][CH_B][CH_C]=1;
		storage_struct.xtalk[6][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[7][CH_A][CH_A]=1;							// position 7
		storage_struct.xtalk[7][CH_B][CH_B]=1;
		storage_struct.xtalk[7][CH_C][CH_C]=1;
		storage_struct.xtalk[7][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[8][CH_C][CH_A]=1;							// position 8
		storage_struct.xtalk[8][CH_D][CH_B]=1;
		storage_struct.xtalk[8][CH_A][CH_C]=1;
		storage_struct.xtalk[8][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[9][CH_B][CH_A]=1;							// position 9
		storage_struct.xtalk[9][CH_D][CH_B]=1;
		storage_struct.xtalk[9][CH_A][CH_C]=1;
		storage_struct.xtalk[9][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[10][CH_C][CH_A]=1;							// position 10
		storage_struct.xtalk[10][CH_A][CH_B]=1;
		storage_struct.xtalk[10][CH_D][CH_C]=1;
		storage_struct.xtalk[10][CH_B][CH_D]=1;
    	
		storage_struct.xtalk[11][CH_B][CH_A]=1;							// position 11
		storage_struct.xtalk[11][CH_A][CH_B]=1;
		storage_struct.xtalk[11][CH_D][CH_C]=1;
		storage_struct.xtalk[11][CH_C][CH_D]=1;
    	
		storage_struct.xtalk[12][CH_C][CH_A]=1;							// position 12
		storage_struct.xtalk[12][CH_D][CH_B]=1;
		storage_struct.xtalk[12][CH_B][CH_C]=1;
		storage_struct.xtalk[12][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[13][CH_B][CH_A]=1;							// position 13
		storage_struct.xtalk[13][CH_D][CH_B]=1;
		storage_struct.xtalk[13][CH_C][CH_C]=1;
		storage_struct.xtalk[13][CH_A][CH_D]=1;
    	
		storage_struct.xtalk[14][CH_C][CH_A]=1;							// position 14
		storage_struct.xtalk[14][CH_A][CH_B]=1;
		storage_struct.xtalk[14][CH_B][CH_C]=1;
		storage_struct.xtalk[14][CH_D][CH_D]=1;
    	
		storage_struct.xtalk[15][CH_B][CH_A]=1;							// position 15
		storage_struct.xtalk[15][CH_A][CH_B]=1;
		storage_struct.xtalk[15][CH_C][CH_C]=1;
		storage_struct.xtalk[15][CH_D][CH_D]=1;
}

	return 0;
	
}

//**************************************************************************************************
