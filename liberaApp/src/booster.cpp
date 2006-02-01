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


/* Implementation of Booster data support. */

#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "publish.h"
#include "trigger.h"
#include "hardware.h"
#include "events.h"
#include "waveform.h"

#include "booster.h"


/* The booster long waveform must be long enough to accomodate a complete
 * booster acceleration ramp lasting 100ms at a sample rate of turn-by-turn
 * frequency decimated by 64: this corresponds to 2957 points.
 *    Also, to help with computation of the short waveform, we would like
 * this count to be a multiple of 16.  For the sake of relatively round
 * numbers we take value 3040 and 190 respectively. */
#define LONG_WAVEFORM_SIZE      3040
#define SHORT_WAVEFORM_SIZE     (LONG_WAVEFORM_SIZE/16)





class BOOSTER : I_EVENT
{
public:
    BOOSTER() :
        LongWaveform(LONG_WAVEFORM_SIZE),
        ShortWaveformX(SHORT_WAVEFORM_SIZE),
        ShortWaveformY(SHORT_WAVEFORM_SIZE),
        ShortWaveformS(SHORT_WAVEFORM_SIZE)
    {
        /* Publish the PVs associated with Booster data. */
        Publish_waveform("BN:WFA", LongWaveform.Waveform(0));
        Publish_waveform("BN:WFB", LongWaveform.Waveform(1));
        Publish_waveform("BN:WFC", LongWaveform.Waveform(2));
        Publish_waveform("BN:WFD", LongWaveform.Waveform(3));
        Publish_waveform("BN:WFX", LongWaveform.Waveform(4));
        Publish_waveform("BN:WFY", LongWaveform.Waveform(5));
        Publish_waveform("BN:WFQ", LongWaveform.Waveform(6));
        Publish_waveform("BN:WFS", LongWaveform.Waveform(7));

        Publish_waveform("BN:WFSX", ShortWaveformX);
        Publish_waveform("BN:WFSY", ShortWaveformY);
        Publish_waveform("BN:WFSS", ShortWaveformS);

        /* Publish the trigger.  This trigger will be signalled whenever our
         * data has updated. */
        Publish_bi("BN:TRIG", Trigger);

        /* Announce our interest in the trigger. */
        RegisterEvent(*this, PRIORITY_BN);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent()
    {
        /* Capture the long waveform and reduce to proper values. */
        LongWaveform.Capture(64);
        LongWaveform.Cordic(10);        // Need to find how many bits needed!
        LongWaveform.ABCDtoXYQS();

        /* Extract the short XY waveforms. */
        ProcessShortWaveform(ShortWaveformX, 4);
        ProcessShortWaveform(ShortWaveformY, 5);
        ProcessShortWaveform(ShortWaveformS, 7);

        /* Let EPICS know there's stuff to read. */
        Trigger.Ready();
    }

    
private:
    /* Extract short waveform information. */
    void ProcessShortWaveform(SIMPLE_WAVEFORM & Waveform, int Index)
    {
        /* For simplicity we grab the raw data to be reduced into a local
         * buffer.  This costs 3040*4 bytes and a little time to copy the
         * memory. */
        int Buffer[LONG_WAVEFORM_SIZE];
        LongWaveform.Read(Index, Buffer, 0, LONG_WAVEFORM_SIZE);
        int * Target = Waveform.Array();
        for (int i = 0; i < SHORT_WAVEFORM_SIZE; i += 1)
        {
            int Total = 0;
            for (int j = 0; j < 16; j ++)
                Total += Buffer[16*i + j] >> 4;
            Target[i] = Total;
        }
    }
    
    LIBERA_WAVEFORM LongWaveform;
    SIMPLE_WAVEFORM ShortWaveformX;
    SIMPLE_WAVEFORM ShortWaveformY;
    SIMPLE_WAVEFORM ShortWaveformS;
    TRIGGER Trigger;
};



BOOSTER * Booster = NULL;

bool InitialiseBooster()
{
    Booster = new BOOSTER();
    return true;
}
