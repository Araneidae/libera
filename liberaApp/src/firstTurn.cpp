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


/* Implementation of First Turn support. */

#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "publish.h"
#include "trigger.h"
#include "hardware.h"
#include "events.h"
#include "waveform.h"
#include "convert.h"

#include "firstTurn.h"


/* Length of waveform examined for first turn data.  In reality we're only
 * interested in a window of about 4 to 8 points, but a wider window helps
 * with finding the trigger point. */
#define WAVEFORM_LENGTH 32


/* provides support for "first turn" data.  This uses triggered data to read
 * a short waveform which is then converted into X,Y,S,Q values locally. */

class FIRST_TURN : I_EVENT
{
public:
    FIRST_TURN(size_t WaveformSize) :
        Waveform(WaveformSize)
    {
        /* Sensible defaults for offset and length.  Must be bounded to lie
         * within the waveform. */
        Offset = 10;
        SetLength(5);
        
        /* Publish the PVs associated with First Turn data. */
        
        /* The computed X, Y, Q, S values. */
        Publish_longin("FT:A", Row[0]);
        Publish_longin("FT:B", Row[1]);
        Publish_longin("FT:C", Row[2]);
        Publish_longin("FT:D", Row[3]);
        Publish_ai("FT:X", X);
        Publish_ai("FT:Y", Y);
        Publish_ai("FT:Q", Q);
        Publish_longin("FT:S", Row[7]);
        /* The raw waveforms are provided for help with trigger placement and
         * other diagnostic operations. */
        Publish_waveform("FT:WFA", Waveform.Waveform(0));
        Publish_waveform("FT:WFB", Waveform.Waveform(1));
        Publish_waveform("FT:WFC", Waveform.Waveform(2));
        Publish_waveform("FT:WFD", Waveform.Waveform(3));
        /* Finally the trigger used to notify events.  The database wires this
         * up so that the all the variables above are processed when a trigger
         * has occured.  This code is then responsible for ensuring that all
         * the waveforms are updated before the trigger is updated.   */
        Publish_bi("FT:TRIG", Trigger);

        /* Also publish access to the offset and length controls. */
        Publish_longin("FT:OFF", Offset);
        Publish_longin("FT:LEN", Length);
        PUBLISH_METHOD(longout, "FT:OFF", SetOffset);
        PUBLISH_METHOD(longout, "FT:LEN", SetLength);

        /* Announce our interest in the trigger event from libera. */
        RegisterEvent(*this, PRIORITY_FT);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent()
    {
        Waveform.Capture(1);
        Waveform.Cordic(8);

        /* Prepare the button data into a LIBERA_ROW that we can use standard
         * conversion functions.  Average the waveforms to compute A-D. */
        Row[0] = Average(0);    /* Button A */
        Row[1] = Average(1);    /*        B */
        Row[2] = Average(2);    /*        C */
        Row[3] = Average(3);    /*        D */
        ABCDtoXYQS(&Row, 1);    /* Compute X,Y,Q,S */

        /* For the moment just pull out the positions again and convert
         * directly to mm. */
        X = 1e-6 * Row[4];
        Y = 1e-6 * Row[5];
        Q = 1e-6 * Row[6];

        /* Finally tell EPICS there's stuff to read. */
        Trigger.Ready();
    }

    
private:
    FIRST_TURN();  // Required for PUBLISH_METHOD macros to work!

    /* Function for computing an average button value.  The Index selects
     * which button to average. */
    int Average(int Index)
    {
        int Data[WAVEFORM_LENGTH];
        Waveform.Read(Index, Data, 0, WAVEFORM_LENGTH);

        int total = 0;
        for (int i = 0; i < Length; i ++)
            /* We divide each value by 32 to avoid overflow.  Copes with the
             * case Length = total Length = 32. */
            total += Data[Offset + i] >> 5;
        return ((long long) total * AverageScale) >> 24;
    }

    /* Access methods for offset and length. */
    bool SetOffset(int newOffset)
    {
        if (0 <= newOffset  &&  newOffset + Length <= WAVEFORM_LENGTH)
        {
            Offset = newOffset;
            return true;
        }
        else
        {
            printf("Offset setting %d too large (length = %d)\n",
                newOffset, Length);
            return false;
        }
    }

    bool SetLength(int newLength)
    {
        if (0 < newLength  &&  Offset + newLength <= WAVEFORM_LENGTH)
        {
            Length = newLength;
            AverageScale = (1 << 29) / Length;
            return true;
        }
        else
        {
            printf("Length setting %d too large (offset = %d)\n",
                newLength, Offset);
            return false;
        }
    }
    
    LIBERA_WAVEFORM Waveform;

    /* Control variables for averaging. */
    int Offset;
    int Length;
    /* This is set to 2^29/Length: this allows us to compute the average of
     * Length points with a simple double-word multiplication. */
    int AverageScale;
    
    /* Computed state.  The Row is averaged from the selection of points, and
     * the appropriate elements are published to epics. */
    LIBERA_ROW Row;
    double X, Y, Q;

    /* This is used to inform EPICS of our updated. */
    TRIGGER Trigger;
};



FIRST_TURN * FirstTurn = NULL;

bool InitialiseFirstTurn()
{
    FirstTurn = new FIRST_TURN(32);
    return true;
}
