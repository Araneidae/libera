/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009  Michael Abbott, Diamond Light Source Ltd.
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


/* These globals control the operation of the EPICS driver. */
extern int DecimationFactor;        // Samples per revolution
extern int FA_FIR_Decimation;       // FA FIR decimation factor
extern int FA_DecimationFirLength;  // Length of FIR FA decimation filter
extern bool LiberaBrilliance;       // Brilliance option present
extern bool OldBrillianceApi;       // Old Brilliance attenuator interface
extern bool FastFeedbackFeature;    // Fast feedback option detected
extern bool DlsFpgaFeatures;        // DLS FPGA installed
extern bool MafFeaturePresent;      // Moving Average turn-by-turn filter
extern bool ItechMaxAdcPresent;     // i-Tech version of MAX_ADC register
extern bool Version2FpgaPresent;    // Libera 2.00+ FPGA features present
extern bool SecondaryInterlock;     // Extra interlock controls
extern bool FAPayloadSelection;     // FA Payload selection


bool InitialiseVersions(void);
void StartupMessage(void);
