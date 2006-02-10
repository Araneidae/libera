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


/* Libera events handling interface. */

bool InitialiseEventReceiver();

void TerminateEventReceiver();


/* Libera event registration.  Note that this callback method will be called
 * on a separate event receiver thread. */
class I_EVENT
{
public:
    virtual void OnEvent() = 0;
};

void RegisterTriggerEvent(I_EVENT &Event, int Priority);
void RegisterSlowAcquisitionEvent(I_EVENT &Event);



/* Sensible event priorities for Libera event subscribers. */

#define PRIORITY_FT     0       // First Turn should go first
#define PRIORITY_SA     1       // Slow acquisition gets second place
#define PRIORITY_BN     10      // Decimated booster mode
#define PRIORITY_TT     1000    // Turn-by-turn takes forever and goes last
