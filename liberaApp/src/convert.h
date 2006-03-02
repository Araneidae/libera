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

/* Raw IQ data.  This is identical in layout to the LIBERA_ROW structure. */
struct IQ_ROW
{
    int AI, AQ;
    int BI, BQ;
    int CI, CQ;
    int DI, DQ;
};

/* Button values. */
typedef SA_DATA         ABCD_ROW;

/* Computed X, Y values in nm. */
struct XYQS_ROW
{
    int X, Y, Q, S;
};

/* X, Y values scaled to mm. */
struct XYQSmm_ROW
{
    double X, Y, Q;  // SL;  Maybe at some point...
    int S;
};

/* Some field identifiers used for indexes into the structures above. */
#ifdef offsetof
/* At present we only use offsets into ABCD. */
#define FIELD_A         offsetof(ABCD_ROW, A)
#define FIELD_B         offsetof(ABCD_ROW, B)
#define FIELD_C         offsetof(ABCD_ROW, C)
#define FIELD_D         offsetof(ABCD_ROW, D)
#endif


/* Converts Count rows of IQ data into ABCD format by applying Cordic
 * conversion on each I,Q pair. */
void IQtoABCD(const IQ_ROW *IQ, ABCD_ROW *ABCD, int Count);

/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function. */
void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, int Count);

/* Rescales one row XYQS data from nm to mm. */
void XYQStomm(const XYQS_ROW &XYQSnm, XYQSmm_ROW &XYQSmm);

/* Combined single row ABCD to XYQSmm conversion. */
void ABCDtoXYQSmm(const ABCD_ROW &ABCD, XYQSmm_ROW &XYQSmm);


/* Publishes conversion control PVs to EPICS. */
bool InitialiseConvert();
