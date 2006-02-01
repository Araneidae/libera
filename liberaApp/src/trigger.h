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


/* Simple trigger event notification to EPICS. */


/* A simple trigger class designed to publish I/O Intr events to EPICS. */

class TRIGGER : public I_bi
{
public:
    TRIGGER(bool InitialValue=true);

    /* This method is used to signal EPICS that this trigger is ready. */
    void Ready();

    /* This changes the trigger value and also signals EPICS. */
    void Write(bool NewValue);

    /* Reads the currently set trigger value. */
    bool Read() { return Value; }

private:
    bool read(bool &Value);
    bool EnableIoIntr(I_INTR & Intr);

    /* Callback used to notify trigger event to EPICS. */
    I_INTR * iIntr;
    /* Internal value. */
    bool Value;
};



