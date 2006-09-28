/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2006  Michael Abbott, Diamond Light Source Ltd.
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
 */


/* Libera position calculations and conversions. */

/* The data is returned from LIBERA as a row of 8 integers representing sin
 * and cos pairs for each button reading.  Before use this will need to be
 * reduced to button magnitude and X,Y,S,Q values.
 *
 * The raw data consists of cos/sin pairs for each button as follows:
 *      0, 1    A * (cos,sin)
 *      2, 3    B * (cos,sin)
 *      4, 5    C * (cos,sin)
 *      6, 7    D * (cos,sin)
 *
 * A CORDIC algorithm is used to rapidly compute button signal magnitudes for
 * buttons A to D.  These button values are then used to compute X and Y
 * positions as well total intensity S and a "skew" factor Q.
 *
 * All arithmetic is done with 32 bit integers and with attention paid at all
 * times to performance: these conversions are performed *frequently*! 
 * The final X,Y values are written in units of nm: this gives both an
 * adequate dynamic range (several metres!) and precision.
 * 
 * The generic data processing chain consists of the following steps:
 * 
 *         Cordic       Convert          Scale
 *      IQ ------> ABCD ------> XYSQ(nm) -----> XYSQ(mm)
 *
 */



/* Some field identifiers used for indexes into the structures above. */
#ifdef offsetof
/* At present we only use offsets into ABCD. */
#define FIELD_A         offsetof(ABCD_ROW, A)
#define FIELD_B         offsetof(ABCD_ROW, B)
#define FIELD_C         offsetof(ABCD_ROW, C)
#define FIELD_D         offsetof(ABCD_ROW, D)
#endif


/* All dB values are scaled by 1e6: this is a fairly standard scaling for
 * values intended for transmission through an ai/ao record. */
#define DB_SCALE        1000000

/* Attenuation for sensible signal level at input power of 0dBm, about 45
 * dBm.  This is a reference point for the scaling factor passed to
 * ComputeScaledCurrent(), below. */
#define A_0                     (45 * DB_SCALE)         // 45 dBm


/* Converts Count rows of IQ data into ABCD format by applying Cordic
 * conversion on each I,Q pair. */
void IQtoABCD(const IQ_ROW *IQ, ABCD_ROW *ABCD, int Count);

/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function. */
void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, int Count);


/* Gain correction on a single column of data from a single channel.  Note
 * that gain conversion is performed on RF board channels, not on buttons, so
 * the channel permutation needs to be taken into account before performing
 * this correction. */
void GainCorrect(int Channel, int *Column, int Count);


/* Returns the corrected settings of the attenuators.  The returned value is
 * scaled by 1e6 to provide a dB value suitable for direct display. */
int ReadCorrectedAttenuation();


/* Computes the beam current corresponding to the given readout Intensity.
 * The given IntensityScale should correspond to the nominal intensity
 * reading at 0dBm input and A_0 attenuator setting.   The value returned is
 * given by:
 *                          A - A_0
 *                          -------
 *                            20
 *      I = K  * S * K  * 10
 *           I        S
 * where
 *      K_I = beam current at 0dBm input power
 *      I   = computed scaled current
 *      K_S = IntensityScale
 *      S   = Intensity
 *      A   = current attenuator setting
 *      A_0 = nominal 0dBm attenuator settings
 *
 * Given that the current scale is normally in units of 10nA, ie 10^-8 A (so
 * allowing a full scale of 20A beam current) then so is the scaled current
 * returned by this routine. */
class PMFP;
int ComputeScaledCurrent(const PMFP & IntensityScale, int Intensity);

/* Updates attenuators to new value.  Takes immediate effect. */
void UpdateAttenuation(int NewAttenuation);

/* Publishes conversion control PVs to EPICS. */
bool InitialiseConvert(
    int _LmtdPrescale, int _SamplesPerTurn, int _BunchesPerTurn);
