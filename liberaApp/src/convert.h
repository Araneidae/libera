/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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

/* Some field identifiers used for indexes into the structures described
 * above. */

/* Offsets into ABCD_ROW. */
#define FIELD_A         offsetof(ABCD_ROW, A)
#define FIELD_B         offsetof(ABCD_ROW, B)
#define FIELD_C         offsetof(ABCD_ROW, C)
#define FIELD_D         offsetof(ABCD_ROW, D)

/* Offsets into XYQS_ROW. */
#define FIELD_X         offsetof(XYQS_ROW, X)
#define FIELD_Y         offsetof(XYQS_ROW, Y)

/* Dual of offsetof macro, used to reference a selected field. */
#define use_offset(Type, Struct, Field) \
    (*(Type *)((char *)&(Struct) + (Field)))


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

/* Publishes conversion control PVs to EPICS. */
bool InitialiseConvert();
