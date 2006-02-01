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
 * After Cordic reduction the absolute button values are written into the
 * first four slots thus:
 *      0       A
 *      1       B
 *      2       C
 *      3       D
 * The Cordic scaling factor (approximately 0.8284 after prescaling) is
 * included in these values, as it will vanish at the next stage.
 *
 * The next stage is to compute X and Y values together with S and Q.  The
 * values are placed in the remaining four slots thus:
 *      4       X
 *      5       Y
 *      6       Q
 *      7       S
 *
 * At all stages the scalling of these values is managed to ensure that as
 * many bits of precision are preserved without overflowing a 32-bit int.
 * The final X,Y values will be written in units of nm. */


/* Generic utility for computing X,Y,Q from a Scaling factor K, the computed
 * Delta M and a total Intensity S.  This computes
 *      (K * M) / S .
 * There is an assumption (which is *not* checked) that the Scaling factor is
 * strictly smaller than 2^25: this corresponds to an overall scaling factor
 * of 32mm.  If K is too large then the results returned will be rubbish, as
 * internal overflow will occur. */
int DeltaToPosition(int Scaling, int Delta, int Intensity);

/* Converts rows as read directly from Libera into button intensities.  On
 * input have rows thus:
 *                      Cos     Sin
 *      Button A         0       1              Number is index into Rows[i]
 *             B         2       3
 *             C         4       5
 *             D         6       7
 *
 * On exit the first four entries in each row have been translated into button
 * intensities. */
void SinCosToABCD(LIBERA_ROW * Rows, size_t Length, int Iterations);


/* Computes XYSQ values from ABCD button values. */
void ABCDtoXYQS(LIBERA_ROW * Row, size_t Length);


/* Publishes conversion control PVs to EPICS. */
bool InitialiseConvert();
