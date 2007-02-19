// $Id: dsc_set.c,v 1.2 2006/06/02 08:41:34 primoz Exp $

//! \file dsc_set.c
//! Implements DSC-FPGA set utility.

/* Copyright (C) 2004-2006 Instrumentation Technologies */

#define _GNU_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "dsc_fpga.h"


// compile for ARM: arm-linux-gcc -Wall -g -o dsc_set -lm dsc_fpga.c dsc_set.c

#define ATT1 31
#define ATT2 31

//unsigned int pos_array[]={0,0,3,3,12,12,15,15};					// switch 4 positions - Peter

	unsigned int pos_array_ABCD[]={3,7,15,11,0,4,12,8};				// switch	positions - BoSo - ABCD 
	unsigned int pos_array_ABDC[]={3,1,0,2,14,12,13,15};			// switch 8 positions - BoSo - ABDC 
	int No_positions = 8;


unsigned int ext_sw_pattern[MAX_SW_PATTERN];					// external switching pattern

unsigned char atts[]={ATT1, ATT2, ATT1, ATT2, ATT1, ATT2, ATT1, ATT2};	// for 0dBm at the Libera input - 8dBm at RF generator

unsigned char de_switch_tab[64]= {  							// input for channels order A, B, C, D !!!
	CH_D, CH_C, CH_B, CH_A,
	CH_D, CH_B, CH_C, CH_A,
	CH_A, CH_C, CH_B, CH_D,
	CH_A, CH_B, CH_C, CH_D,
	CH_C, CH_D, CH_B, CH_A,
	CH_C, CH_B, CH_D, CH_A,
	CH_A, CH_D, CH_B, CH_C,
	CH_A, CH_B, CH_D, CH_C,
	CH_D, CH_C, CH_A, CH_B,
	CH_D, CH_A, CH_C, CH_B,
	CH_B, CH_C, CH_A, CH_D,
	CH_B, CH_A, CH_C, CH_D,
	CH_C, CH_D, CH_A, CH_B,
	CH_C, CH_A, CH_D, CH_B,
	CH_B, CH_D, CH_A, CH_C,
	CH_B, CH_A, CH_D, CH_C };
  


int main(int argc, char *argv[]) {
	
	int help = 0;
	int c;
	char * s_fixed_pos = NULL;	
	char * s_external_pattern = NULL;		
	char * s_ampcomp = NULL;	
	char * s_xtalk = NULL;
	char * s_phase = NULL;	
		
	int i;
	FILE *in_file_ptr = NULL;

	char * line;
    size_t len = 0;
    unsigned int position, channel, frm_channel, to_channel;
    float compensation;
    int line_no;
    int array_pos;
    
    int channel_order = 0;						// normally ABCD order
    int ana2dig_delay = 40;						// analog switches 2 digital delay
    int ADCRB_trigger = pos_array_ABCD[0];		// ADCRB trigger source (delay set to the beginning of new position state)
    unsigned int fRF = 499654000;
	unsigned int harmonic = 936;    
    unsigned int decimation = 220;
    unsigned int marker = pos_array_ABCD[0];
    signed int val_ATT1 = -1;
    signed int val_ATT2 = -1;    
    
//---------------------------------------------------------------------------------------------------    	
	while ((c=getopt(argc, argv, "a:d:f:hi:j:k:m:n:o:p:r:s:t:x:")) != -1)
		switch (c) {
			case 'a':
				s_ampcomp = optarg;
				break;
				
			case 'd':
				ana2dig_delay = atoi(optarg) & 1023;
				break;
				
			case 'f':
				s_phase = optarg;
				break;				
				
			case 'h':
				help = 1;
				break;
				
			case 'i':
				val_ATT1 = atoi(optarg) & 0x1f;
				break;
				
			case 'j':
				val_ATT2 = atoi(optarg) & 0x1f;
				break;
				
			case 'k':
				harmonic = atoi(optarg) & 4095;
				break;
				
			case 'm':
				marker = atoi(optarg) & 0x0f;
				break;				
				
			case 'n':
				decimation = atoi(optarg) & 1023;
				break;

			case 'o':
				channel_order = atoi(optarg) & 0x01;
				break;
								
			case 'p':
				s_fixed_pos =optarg;
				break;
				
			case 'r':
				fRF = atoi(optarg);
				break;
				
			case 's':
				s_external_pattern =optarg;
				break;
				
			case 't':
				ADCRB_trigger = atoi(optarg) & 15;
				break;
				
			case 'x':
				s_xtalk = optarg;
				break;

		}
		
	if (help) {
		printf("usage: dsc_test [options]\n");
		printf("options:\n");
		
		#ifdef __DEVEL
		printf("[-a filename] amplitude compensation coefficients file\n");
		#endif // __DEVEL
		
		printf("[-d samples] delay in RF board (ADC samples)\n");
		
		#ifdef __DEVEL		
		printf("[-f filename] phase compensation coefficients filename\n");
		#endif // __DEVEL		
		
		printf("[-h] help\n");
		printf("[-i attenuation] ATT1 value in all RF chains (dB)\n");
		printf("[-j attenuation] ATT2 value in all RF chains (dB)\n");						
		
		#ifdef __DEVEL
		printf("[-k harmonic] harmonic number (fRF to TBT rate)\n");
		printf("[-m position] TBT rate marker position\n");		
		printf("[-n decimation] decimation (fsamp to TBT rate)\n");
		#endif // __DEVEL
		
		printf("[-o order] input (RF) channel order (0=ABCD, 1=ABDC)\n");
		printf("[-p position] fixed switch position\n");
		
		#ifdef __DEVEL
		printf("[-r frequency] input RF frequency (Hz)\n");
		#endif // __DEVEL
		
		printf("[-s filename] switching pattern file\n\n");
		
		#ifdef __DEVEL
		printf("[-x filename] crosstalk compensation coefficients filename\n");
		#endif // __DEVEL
				
		printf("if no options are specified:\n");
		printf("- both ATTs are set to 31 dB\n");
		printf("- input switch is rotating through 8 positions\n");
		printf("- input channel order is ABCD\n");				
		printf("- RF board delay is set to 40\n");				
		
		return 0;
	}	
	
	// optional fixed position
	if (s_fixed_pos != NULL) {
		No_positions = 1;
		pos_array_ABCD[0] = atoi(s_fixed_pos) & 0x0f;
		pos_array_ABDC[0] = atoi(s_fixed_pos) & 0x0f;				
	}
	
	// reading input level for attenuator settings
		if (val_ATT1 != -1)
			for (i=0; i<4; i++) 
				atts[i*2] = val_ATT1;

		if (val_ATT2 !=-1)
			for (i=0; i<4; i++) 
				atts[i*2+1] = val_ATT2;



//---------------------------------------------------------------------------------------------------
	// reading file that contains switching pattern
	line = NULL;
	if (s_external_pattern !=NULL) {
		in_file_ptr = fopen(s_external_pattern, "r");
		if (in_file_ptr != NULL) {
			printf("Switching pattern file content:\n");
			printf("POS\n");
			line_no = 1;
			array_pos = 0;
			while (getline(&line, &len, in_file_ptr) != -1) {
				if (line[0] != '#') {																// not a comment line
					if (sscanf (line, "%d", &position) == 1) {										// properly read
						printf("%d\n", position);

						ext_sw_pattern[array_pos] = position & 0x0f;
						array_pos ++;						
					}
					else																			// wrong number of parameters
						printf ("Error in line %d\n",line_no+1);
				}
				line_no ++;				
			}
			if (line)
				free(line);
				
			if (array_pos)																			// if any position from file was read
 				DSC_Set_sw_pattern (ext_sw_pattern, array_pos);
 				
     		fclose(in_file_ptr);
	     }
    	 else {
     		printf("Switching pattern file not found\n");
     		s_external_pattern = NULL; 																// to indicate file was not found
     	}
     }

//---------------------------------------------------------------------------------------------------				
	// Write initial settings to the DSC structure
	DSC_init(fRF/harmonic*decimation, fRF*(1-(float)decimation/harmonic *( harmonic/decimation)) , channel_order);
	
	
	// write settings entered as command line parameters
	// switching pattern
	if (s_external_pattern == NULL)
		if(channel_order == 0)
			DSC_Set_sw_pattern(pos_array_ABCD, No_positions);
		else
			DSC_Set_sw_pattern(pos_array_ABDC, No_positions);
	else
		DSC_Set_sw_pattern(ext_sw_pattern, No_positions);		
		
	// analog 2 digital delay
	DSC_Set_ana2dig_delay (ana2dig_delay);
	
	// attenuators
	DSC_set_att(atts);
	
	// ADCRB trigger
	#ifdef __DEVEL
		DSC_Set_ADCRB_trigger (ADCRB_trigger, 0x1a00, 1);
	#else
		DSC_Set_ADCRB_trigger (ADCRB_trigger, 0x1a00, 0);
	#endif
	
	// TBT marker
	DSC_Set_TBT_marker (marker,0x1a00);

//---------------------------------------------------------------------------------------------------

	// reading file with xtalk compensation coefficients
	line = NULL;
	if (s_xtalk !=NULL) {
		in_file_ptr = fopen(s_xtalk, "r");
		if (in_file_ptr != NULL) {
			printf("Xtalk compensation file content:\n");
			printf("POS FRM TO VAL\n");
			line_no = 1;
			while (getline(&line, &len, in_file_ptr) != -1) {
				if (line[0] != '#') {																// not a comment line
					if (sscanf (line, "%d %d %d %f", &position, &frm_channel, &to_channel, &compensation ) == 4) { // properly read
						printf("%3d %3d %2d %f\n", position, frm_channel, to_channel, compensation);
						DSC_Set_xtalk (position, frm_channel, to_channel, compensation);
					}
					else																			// wrong number of parameters
						printf ("Error in line %d\n",line_no+1);
				}
				line_no ++;				
			}
			if (line)
				free(line);
				
     		fclose(in_file_ptr);
	     }
    	 else {
     		printf("Xtalk compensation coefficients file not found\n");
     		s_xtalk = NULL; 																// to indicate file was not found
     	}
     }


//---------------------------------------------------------------------------------------------------

	// reading file with phase compensation coefficients
	line = NULL;
	if (s_phase !=NULL) {
		in_file_ptr = fopen(s_phase, "r");
		if (in_file_ptr != NULL) {
			printf("Phase compensation file content:\n");
			printf("POS CH PHASE\n");
			line_no = 1;
			while (getline(&line, &len, in_file_ptr) != -1) {
				if (line[0] != '#') {																// not a comment line
					if (sscanf (line, "%d %d %f", &position, &channel, &compensation ) == 3) { 		// properly read
						printf("%3d %2d %f\n", position, channel, compensation);
						DSC_Set_phase (position, channel, compensation);
					}
					else																			// wrong number of parameters
						printf ("Error in line %d\n",line_no+1);
				}
				line_no ++;				
			}
			if (line)
				free(line);
				
     		fclose(in_file_ptr);
	     }
    	 else {
     		printf("Phase compensation coefficients file not found\n");
     		s_phase = NULL; 																// to indicate file was not found
     	}
     }

//---------------------------------------------------------------------------------------------------

	// reading file that contains amplitude compensation coefficients
	line = NULL;
	if (s_ampcomp !=NULL) {
		in_file_ptr = fopen(s_ampcomp, "r");
		if (in_file_ptr != NULL) {
			printf("Amplitude compensation file content:\n");
			printf("POS CH VAL\n");
			line_no = 1;
			while (getline(&line, &len, in_file_ptr) != -1) {
				if (line[0] != '#') {																// not a comment line
					if (sscanf (line, "%d %d %f", &position, &channel, &compensation) == 3) {		// properly read
						printf("%d %d %f\n", position, channel, compensation);
						DSC_Set_gain (position, channel-1, compensation);
						//DSC_Set_xtalk (position, channel-1, de_switch_tab[4 * position + channel-1], compensation);	// TEST: analog compensation @ de-switching						
					}
					else																			// wrong number of parameters
						printf ("Error in line %d\n",line_no);
				}
				line_no ++;
			}
			if (line)
				free(line);
     		fclose(in_file_ptr);
	     }
    	 else
     		printf("Amplitude compensation coefficients file not found\n");
     }
     
//---------------------------------------------------------------------------------------------------
	// aplying setting to the FPGA
	DSC_Apply_all();

    return EXIT_SUCCESS;
}


//---------------------------------------------------------------------------------------------------
