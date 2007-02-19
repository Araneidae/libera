//! \file cordic_DSC.c
//! Implements CORDIC algorithm.

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

#include "cordic_dsc.h"


/* CORDIC level. The number of iterations = CORDIC_DSC_MAXLEVEL + 1. */
#define CORDIC_DSC_MAXLEVEL 11 

#define PI_POLA   1811004864519280600


long long table_atan2[29] = {	  905502432259640320   ,
                                534549298976576450   ,
                                282441168888798110   ,
                                143371547418228450   ,
                                71963988336308048    ,
                                36017075762092180    ,
                                18012932708689206    ,
                                9007016009513623     ,
                                4503576721087964     ,
                                2251796950380271     ,
                                1125899548928888     ,
                                562949908682076      ,
                                281474971118251      ,
                                140737487656277      ,
                                70368744090283       ,
                                35184372077909       ,
                                17592186043051       ,
                                8796093022037        ,
                                4398046511083        ,
                                2199023255549        ,
                                1099511627776        ,
                                549755813888         ,
                                274877906944         ,
                                137438953472         ,
                                68719476736          ,
                                34359738368          ,
                                17179869184          ,
                                8589934592           ,
                                4294967296           };




                          
long long cordic_DSC_phase( int I_input, int Q_input )
{                         
                          
	int I_work, Q_work, I_temp;
	int kkk=0;                
	long long angle_return = 0;
	long long angle_correction = 0;              

	I_work = I_input;
	Q_work = Q_input;
	angle_return = 0;
	angle_correction = 0;
	
	if ( I_work<0 ) {
			I_temp = I_work;
			
			if (Q_work>0){
					I_work = Q_work;
					Q_work = - I_temp;
					angle_correction = PI_POLA;
			}	
			else {
					I_work = - Q_work;
					Q_work = I_temp;
					angle_correction = - PI_POLA;
			}
	}
	
	for(kkk=0; kkk <= CORDIC_DSC_MAXLEVEL; kkk++) {
			I_temp = I_work;
			
			if (Q_work >= 0) {
					I_work = I_work + ( Q_work >> kkk);
					Q_work = Q_work - ( I_temp >> kkk);
					angle_return = angle_return + table_atan2[kkk];
			}
			else {
					I_work = I_work - ( Q_work >> kkk);
					Q_work = Q_work + ( I_temp >> kkk);
					angle_return = angle_return - table_atan2[kkk];
			}
			
	}
	angle_return = angle_return + angle_correction;	
	
	return angle_return;
}

