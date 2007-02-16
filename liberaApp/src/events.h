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
 * on a separate event receiver thread.  The parameter is event specific. */
class I_EVENT
{
public:
    virtual void OnEvent(int Parameter) = 0;
};

void RegisterTriggerEvent(I_EVENT &Event, int Priority);
void RegisterTriggerSetEvent(I_EVENT &Event, int Priority);
void RegisterPostmortemEvent(I_EVENT &Event, int Priority);
void RegisterInterlockEvent(I_EVENT &Event, int Priority);



/* Events are processed in the order below.  Note that these numbers are
 * indexes into an array, and so must be unique and in the appropriate
 * range. */

/* PM */
#define PRIORITY_PM     0       // Postmortem wins, every time.

/* INTERLOCK */
#define PRIORITY_IL     1       // Interlock

/* TRIGSET */
#define PRIORITY_SYNC   2       // Set clock

/* TRIGGET */
#define PRIORITY_TICK   3       // Tick event notification 
#define PRIORITY_FT     4       // First Turn 
#define PRIORITY_TT     5       // Turn-by-turn takes forever but goes early
#define PRIORITY_BN     6       // Decimated booster mode
#define PRIORITY_FR     7       // Free running mode
