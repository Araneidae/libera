// $Id: cordic.c,v 1.7 2006/01/12 08:01:51 miha Exp $

//! \file cordic.c
//! Implements CORDIC algorithm.

/*
CSPI - Control System Programming Interface
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

/*
Implementation note: Public functions have their input parameters validated
in both, debug and nondebug builds. Private functions, on the other hand,
validate input parameters in debug build only.

TAB = 4 spaces.
*/

#include "cordic.h"

/*
 *
 * Table of CORDIC gain (CG) corrections:
 *
 * L    CG                       1/CG                   (1/CG)<<32
 *
 * 0  : 1.41421356237310000000   0.70710678118654700    3037000500
 * 1  : 1.58113883008419000000   0.63245553203367600    2716375826
 * 2  : 1.62980060130066000000   0.61357199107789600    2635271635
 * 3  : 1.64248406575224000000   0.60883391251775200    2614921743
 * 4  : 1.64568891575726000000   0.60764825625616800    2609829388
 * 5  : 1.64649227871248000000   0.60735177014129600    2608555990
 * 6  : 1.64669325427364000000   0.60727764409352600    2608237621
 * 7  : 1.64674350659690000000   0.60725911229889300    2608158028
 * 8  : 1.64675607020488000000   0.60725447933256200    2608138129
 * 9  : 1.64675921113982000000   0.60725332108987500    2608133154
 * 10 : 1.64675999637562000000   0.60725303152913400    2608131911
 * 11 : 1.64676019268469000000   0.60725295913894500    2608131600 (!)
 * 12 : 1.64676024176197000000   0.60725294104139700    2608131522
 * 13 : 1.64676025403129000000   0.60725293651701000    2608131503
 * 14 : 1.64676025709862000000   0.60725293538591400    2608131498
 * 15 : 1.64676025786545000000   0.60725293510313900    2608131497
 * 16 : 1.64676025805716000000   0.60725293503244600    2608131496
 * 17 : 1.64676025810509000000   0.60725293501477200    2608131496
 * 18 : 1.64676025811707000000   0.60725293501035400    2608131496
 * 19 : 1.64676025812007000000   0.60725293500925000    2608131496
 * 20 : 1.64676025812082000000   0.60725293500897300    2608131496
 *
 */

/** CORDIC level. The number of iterations = CORDIC level + 1. */
#define CORDIC_MAXLEVEL 11

/** CORDIC gain correction factor associated with the CORDIC_MAXLEVEL. */
const long long CORDIC_GAIN64 = 2608131600LL;	// (1/CG)<<32

//--------------------------------------------------------------------------

static inline int cordic_correct(int iVx)
{
#if defined(CORDIC_IGNORE_GAIN)
	return iVx;
#else
	return (int)((CORDIC_GAIN64 * iVx) >> 32);
#endif
}

//--------------------------------------------------------------------------

int cordic_amp( int I, int Q )
{
	// See http://www.dspguru.com/info/faqs/cordic.htm for
	// information on the CORDIC algorithm.

	// To calculate the magnitude of a complex number (I,Q) we rotate
	// it to have a phase of zero; then its new "Q" value would be zero,
	// so the magnitude would be given entirely by the new "I" value.

	int tmp_I;
	int L=0;

	if ( I < 0 ) {

		tmp_I = I;

		if ( Q > 0 ) {
				I = Q;
				Q = -tmp_I;     // Rotate by -90 degrees
		}
		else {
				I = -Q;
				Q = tmp_I;      // Rotate by +90 degrees
		}
	}

	for( ; L <= CORDIC_MAXLEVEL; ++L  ) {

		tmp_I = I;

		if ( Q >= 0 ) {

			// Positive phase; do negative rotation
			I += (Q >> L); Q -= (tmp_I >> L);
		}
		else {

			// Negative phase; do positive rotation
			I -= (Q >> L); Q += (tmp_I >> L);
		}
	}

	return cordic_correct(I);	// Compensate the CORDIC gain
}

