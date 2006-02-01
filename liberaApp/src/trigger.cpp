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


/* Simple EPICS I/O Intr implementation through a bi record.  The act of
 * writing a boolean value to the trigger invokes EPICS processing for
 * interested records.
 *
 * This is used as a basic building block for passing Libera events up to the
 * interested EPICS records. */


#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "trigger.h"




TRIGGER::TRIGGER(bool InitialValue)
{
    Value = InitialValue;
    iIntr = NULL;
}

/* This method signals that the trigger is ready. */
void TRIGGER::Ready()
{
    if (iIntr != NULL)
        iIntr->IoIntr();
}

/* Epics support routine: actually, nothing to do! */
bool TRIGGER::read(bool &ValueRead)
{
    ValueRead = Value;
    return true;
}

/* Support I/O Intr functionality: overrides I_bo method. */
bool TRIGGER::EnableIoIntr(I_INTR & Intr)
{
    iIntr = &Intr;
    return true;
}

void TRIGGER::Write(bool NewValue)
{
    Value = NewValue;
    Ready();
}

