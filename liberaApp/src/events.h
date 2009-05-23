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


/* Libera events handling interface. */

bool InitialiseEventReceiver();
void TerminateEventReceiver();


/* Events are processed in the order below.  Note that these numbers are
 * indexes into an array, and so must be unique and in the appropriate
 * range. */
enum PRIORITIES
{
    /* PM */
    PRIORITY_PM,        // Postmortem wins, every time.

    /* INTERLOCK */
    PRIORITY_IL,        // Interlock

    /* TRIGSET */
    PRIORITY_SYNC,      // Set clock

    /* TRIGGET */
    PRIORITY_TICK,      // Tick event notification
    PRIORITY_MS,        // Mean sums calculation
    PRIORITY_FT,        // First Turn 
    PRIORITY_TT,        // Turn-by-turn takes forever but goes early
    PRIORITY_FR,        // Free running mode
    PRIORITY_SC,        // Signal conditioning
    PRIORITY_BN,        // Decimated booster mode

    /* This must always come last (and is not a real priority). */
    HANDLER_TABLE_SIZE
};


/* Libera event registration.  Note that this callback method will be called
 * on a separate event receiver thread.  The parameter is event specific. */
class I_EVENT
{
public:
    virtual void OnEvent(int Parameter) = 0;
};

void RegisterTriggerEvent(I_EVENT &Event, PRIORITIES Priority);
void RegisterTriggerSetEvent(I_EVENT &Event, PRIORITIES Priority);
void RegisterPostmortemEvent(I_EVENT &Event, PRIORITIES Priority);
void RegisterInterlockEvent(I_EVENT &Event, PRIORITIES Priority);
