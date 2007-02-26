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


/* Provide support for postmortem data acquired on receipt of a postmortem
 * trigger. */

#include <stdio.h>
#include <stdlib.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"

#include "postmortem.h"


#define POSTMORTEM_LENGTH       16384


class POSTMORTEM : I_EVENT
{
public:
    POSTMORTEM() :
        WaveformIq(POSTMORTEM_LENGTH),
        WaveformAbcd(POSTMORTEM_LENGTH),
        WaveformXyqs(POSTMORTEM_LENGTH)
    {
        /* Publish all the waveforms and the interlock. */
        WaveformIq.Publish("PM");
        WaveformAbcd.Publish("PM");
        WaveformXyqs.Publish("PM");
        Interlock.Publish("PM", true);
        /* Announce our interest in the postmortem event. */
        RegisterPostmortemEvent(*this, PRIORITY_PM);
    }

    void OnEvent(int)
    {
        /* Wait for EPICS to be ready. */
        Interlock.Wait();

        /* Capture and convert everything. */
        WaveformIq.CapturePostmortem();
        WaveformAbcd.CaptureCordic(WaveformIq);
        WaveformXyqs.CaptureConvert(WaveformAbcd);

        /* Let EPICS know there's stuff to read. */
        Interlock.Ready(WaveformIq.GetTimestamp());
    }

    
private:
    /* Captured and processed waveforms: these three blocks of waveforms are
     * all published to EPICS. */
    IQ_WAVEFORMS WaveformIq;
    ABCD_WAVEFORMS WaveformAbcd;
    XYQS_WAVEFORMS WaveformXyqs;

    /* EPICS interlock. */
    INTERLOCK Interlock;
};



static POSTMORTEM * Postmortem = NULL;

bool InitialisePostmortem()
{
    Postmortem = new POSTMORTEM();
    return true;
}
