/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2007  Michael Abbott, Diamond Light Source Ltd.
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


/* Select either a specific switch or rotating switches. */
bool WriteSwitchState(CSPI_SWITCHMODE Switches);

/* Select signal conditioning mode: one of
 *  SC_MODE_FIXED   use current signal conditioning setting
 *  SC_MODE_UNITY   don't correct signal: pass through unchanged
 *  SC_MODE_AUTO    automatic signal conditioning. */
enum SC_MODE {
    SC_MODE_FIXED,
    SC_MODE_UNITY,
    SC_MODE_AUTO
};
bool WriteDscMode(CSPI_DSCMODE DscMode);

/* Selects the attenuation and updates the signal conditioning state
 * accordingly. */
bool WriteDscAttenuation(int Attenuation);


/* A permutation is a mapping from channel to button. */
typedef unsigned int PERMUTATION[4];
/* Returns the button permutation sequence associated with the current
 * statically set switch position. */
const PERMUTATION & SwitchPermutation();


bool InitialiseSignalConditioning(int Harmonic, int Decimation);
void TerminateSignalConditioning();
