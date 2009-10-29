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

/* Libera attenuation control. */


/* All dB values are scaled by 1e6: this is a fairly standard scaling for
 * values intended for transmission through an ai/ao record. */
#define DB_SCALE        1000000

/* Attenuation for sensible signal level at input power of 0dBm, about 45
 * dBm.  This is a reference point for the scaling factor passed to
 * ComputeScaledCurrent(), below. */
#define A_0                     (45 * DB_SCALE)         // 45 dBm


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


void NotifyMaxAdc(int MaxAdc);

bool InitialiseAttenuation();
