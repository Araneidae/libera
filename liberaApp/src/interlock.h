/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
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


/* Interlock management initialisation. */
bool InitialiseInterlock();

/* Called by slow acquisition at approximately 10Hz to notify current level.
 * This is also used as a clock tick to advance the interlock holdoff state
 * machine. */
void NotifyInterlockCurrent(int Current);

/* Called during configuration to record the state of the global enable flag.
 * This is used to control whether interlocks are enabled. */
bool NotifyInterlockBpmEnable(bool Enabled);

/* Called to notify "Golden orbit" offsets so that interlocks can track the
 * true "nominal" zero even while "golden orbit" offsets are being generated
 * in the hardware. */
void NotifyInterlockOffset(int OffsetX, int OffsetY);

/* This routine is called immediately before performing an operation which
 * can cause a glitch in position: changing attenuators or signal
 * conditioning parameters.  The interlock is immediately disabled for a
 * preset period. */
void HoldoffInterlock();
