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
#include "convert.h"
#include "waveform.h"
#include "support.h"

#include "firstTurn.h"



/* provides support for "first turn" data.  This uses triggered data to read
 * a short waveform which is then converted into X,Y,S,Q values locally. */

class FIRST_TURN : I_EVENT
{
public:
    FIRST_TURN(size_t WaveformSize) 
    {
        /* Sensible defaults for offset and length.  Must be bounded to lie
         * within the waveform.
         *    By default we set the window to cover approximately two bunches
         * at booster clock frequency.  This allows a sensible signal to be
         * read without tight adjustment of the timing.  At the Libera sample
         * clock rate of approximately 117MHz, and with four raw ADC points
         * per processed sample, each point corresponds to approximately
         * 34ns. */
        Offset = 5;
        SetLength(31);
        
        /* Publish the PVs associated with First Turn data. */
        
        /* Computed button averages and associated button values. */
        Publish_ABCD("FT", ABCD);
        Publish_XYQS("FT", XYQS);

        /* The computed maximum ADC values (used to detect overflow). */
        Publish_longin("FT:MAXADC", MaxAdc);

        /* The raw waveforms allow access to the completely unprocessed data
         * as read from the ADC.  This is only useful for low level
         * investigations. */
        Publish_waveform("FT:RAWA", AdcWaveform.RawWaveform(0));
        Publish_waveform("FT:RAWB", AdcWaveform.RawWaveform(1));
        Publish_waveform("FT:RAWC", AdcWaveform.RawWaveform(2));
        Publish_waveform("FT:RAWD", AdcWaveform.RawWaveform(3));

        /* Each of these waveforms provides an intensity profile of the
         * electron beam pickup (button) intensity at a bandwidth of about
         * 10MHz with 256 samples at 34ns intervals (covering 8.7 us). */
        Publish_waveform("FT:WFA",  AdcWaveform.Waveform(0));
        Publish_waveform("FT:WFB",  AdcWaveform.Waveform(1));
        Publish_waveform("FT:WFC",  AdcWaveform.Waveform(2));
        Publish_waveform("FT:WFD",  AdcWaveform.Waveform(3));
        
        /* Finally the trigger used to notify events.  The database wires this
         * up so that the all the variables above are processed when a trigger
         * has occured.  This code is then responsible for ensuring that all
         * the waveforms are updated before the trigger is updated.   */
        Interlock.Publish("FT");

        /* Also publish access to the offset and length controls for the
         * averaging window. */
        Publish_longin("FT:OFF", Offset);
        Publish_longin("FT:LEN", Length);
        PUBLISH_METHOD(longout, "FT:OFF", SetOffset);
        PUBLISH_METHOD(longout, "FT:LEN", SetLength);

        /* Announce our interest in the trigger event from libera. */
        RegisterTriggerEvent(*this, PRIORITY_FT);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent()
    {
        Interlock.Wait();
        
        /* Capture a fresh ADC waveform and compute the maximum raw ADC
         * button value: this allows input overload to be detected. */
        AdcWaveform.Capture();
        MaxAdc = Maximum();

        /* Compute the button values as an average within the selected window
         * of the processed ADC data. */
        ABCD.A = Average(0);
        ABCD.B = Average(1);
        ABCD.C = Average(2);
        ABCD.D = Average(3);
        /* Convert button values to XYQS values. */
        ABCDtoXYQSmm(ABCD, XYQS);

        /* Finally tell EPICS there's stuff to read. */
        Interlock.Ready();
    }

    
private:
    FIRST_TURN();  // Required for PUBLISH_METHOD macros to work!

    /* Function for computing an average button value.  The Index selects
     * which button to average. */
    int Average(int Index)
    {
        int * Data = AdcWaveform.Array(Index);

        int Total = 0;
        for (int i = 0; i < Length; i ++)
            /* Prescale data as we accumulate to ensure we don't overflow.
             * We know that each data point is <2^30 (see the Capture()
             * method of ADC_WAVEFORM), and there can be at most 2^8 points,
             * so scaling by 2^-7 is safe. */
            Total += Data[Offset + i] >> 7;
        /* We know that:
         *      AverageScale = 2^30/Length
         *      Total = 2^-7 * Sum
         * so to compute the desired average Sum/Length we compute
         *      2^-23 * Total * AverageScale. */
        return ((long long) Total * AverageScale) >> 23;
    }

    /* Accumulates maximum raw ADC value over entire ADC waveform. */
    int Maximum()
    {
        int Maximum = 0;
        for (int Index = 0; Index < 4; Index ++)
        {
            int * Data = AdcWaveform.RawArray(Index);
            for (int i = 0; i < ADC_LENGTH; i ++)
            {
                int x = Data[i];
                if (x > Maximum)   Maximum = x;
                if (-x > Maximum)  Maximum = -x;
            }
        }
        return Maximum;
    }

    /* Access methods for offset and length. */
    bool SetOffset(int newOffset)
    {
        if (0 <= newOffset  &&  newOffset + Length <= ADC_LENGTH/4)
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
        if (0 < newLength  &&  Offset + newLength <= ADC_LENGTH/4)
        {
            Length = newLength;
            /* We compute the average scale as 2^30/Length so that we can
             * compute the average by a simple multiplication. */
            AverageScale = (1 << 30) / Length;
            return true;
        }
        else
        {
            printf("Length setting %d too large (offset = %d)\n",
                newLength, Offset);
            return false;
        }
    }
    
    ADC_WAVEFORM AdcWaveform;

    /* Control variables for averaging defining the offset into processed ADC
     * buffer and the length of the averaging window. */
    int Offset;
    int Length;
    /* This is set to 2^29/Length: this allows us to compute the average of
     * Length points with a simple double-word multiplication. */
    int AverageScale;
    
    /* Computed state.  The button values are averaged from the selection of
     * points, and the appropriate elements are published to epics. */
    ABCD_ROW ABCD;
    XYQSmm_ROW XYQS;

    /* Maximum raw ADC across all four buttons. */
    int MaxAdc;

    /* Epics trigger and interlock. */
    INTERLOCK Interlock;
};



FIRST_TURN * FirstTurn = NULL;

bool InitialiseFirstTurn()
{
    FirstTurn = new FIRST_TURN(32);
    return true;
}
