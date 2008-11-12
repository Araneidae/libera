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


/* Select either a specific switch or rotating switches.  If AutoSwitch is
 * True then the ManualSwitch argument is ignored. */
void WriteAutoSwitches(bool AutoSwitch);
void WriteManualSwitches(int ManualSwitch);

/* Select signal conditioning mode. */
enum SC_MODE {
    SC_MODE_FIXED,  // Use current signal conditioning setting
    SC_MODE_UNITY,  // Don't correct signal: pass through unchanged
    SC_MODE_AUTO    // Automatic signal conditioning. 
};
void WriteScMode(SC_MODE ScMode);

/* Writes the attenuation with appropriate interlocking with signal
 * conditioning.  In particular, if signal conditioning is operational then
 * the conditioning filter will be reset as the attenuation is written. */
bool ScWriteAttenuation(int Attenuation);

/* Returns the maximum attenuation value: this is architecture dependent, and
 * can be 62 or 31 depending on whether Libera Electron or Brilliance is
 * present. */
int MaximumAttenuation();


/* A permutation is a mapping from channel to button: to be precise, if p is
 * the current permutation then button b (0..3 corresponding to A..D) was
 * processed by ADC channel p[b]. */
typedef unsigned int PERMUTATION[4];
/* Returns the button permutation sequence associated with the current
 * statically set switch position.  If automatic switches are currently
 * selected then the value returned is not meaningful. */
const PERMUTATION & SwitchPermutation();


bool InitialiseSignalConditioning(
    int Harmonic, int Decimation, int TurnsPerSwitch);
void TerminateSignalConditioning();
