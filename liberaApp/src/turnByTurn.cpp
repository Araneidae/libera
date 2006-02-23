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


/* Provide support for very long "turn by turn" data.  This data is triggered,
 * but only as a single shot on demand. */

#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "publish.h"
#include "trigger.h"
#include "hardware.h"
#include "events.h"
#include "waveform.h"

#include "turnByTurn.h"



class TURN_BY_TURN : I_EVENT
{
public:
    TURN_BY_TURN(int LongWaveformLength, int ShortWaveformLength) :
        LongWaveformLength(LongWaveformLength),
        ShortWaveformLength(ShortWaveformLength),
        LongWaveform(LongWaveformLength),
        ShortWaveform(ShortWaveformLength),
        IqWaveform(ShortWaveformLength),
        LongTrigger(false)
    {
        ShortOffset = 0;
        /* Make the default waveform length something more reasonable. */
        LongWaveform.SetLength(ShortWaveformLength);
        /* Don't trigger until asked to. */
        Armed = false;
        
        /* Publish the PVs associated with Turn by Turn data. */

        /* The basic windowed waveform views on the entire turn by turn
         * buffer.  Each of these provides a view of a sub-array of the
         * captured waveform, with offset and length controlled by the OFFSET
         * and LENGTH fields. */
        Publish_waveform("TT:WFA", ShortWaveform.Waveform(0));
        Publish_waveform("TT:WFB", ShortWaveform.Waveform(1));
        Publish_waveform("TT:WFC", ShortWaveform.Waveform(2));
        Publish_waveform("TT:WFD", ShortWaveform.Waveform(3));
        Publish_waveform("TT:WFX", ShortWaveform.Waveform(4));
        Publish_waveform("TT:WFY", ShortWaveform.Waveform(5));
        Publish_waveform("TT:WFQ", ShortWaveform.Waveform(6));
        Publish_waveform("TT:WFS", ShortWaveform.Waveform(7));

        /* Two waveforms providing access to the raw I and Q turn by turn
         * data for each button. */
        Publish_waveform("TT:WFAI", IqWaveform.Waveform(0));
        Publish_waveform("TT:WFAQ", IqWaveform.Waveform(1));
        Publish_waveform("TT:WFBI", IqWaveform.Waveform(2));
        Publish_waveform("TT:WFBQ", IqWaveform.Waveform(3));
        Publish_waveform("TT:WFCI", IqWaveform.Waveform(4));
        Publish_waveform("TT:WFCQ", IqWaveform.Waveform(5));
        Publish_waveform("TT:WFDI", IqWaveform.Waveform(6));
        Publish_waveform("TT:WFDQ", IqWaveform.Waveform(7));

        /* Control fields for managing capture and readout. */
        PUBLISH_METHOD(longout, "TT:CAPLEN", SetCaptureLength);
        PUBLISH_METHOD(longin,  "TT:CAPLEN", GetCaptureLength);
        PUBLISH_METHOD(longin,  "TT:CAPTURED", GetCapturedLength);
        PUBLISH_METHOD(longout, "TT:OFFSET", SetReadoutOffset);
        Publish_longin("TT:OFFSET", ShortOffset);
        PUBLISH_METHOD(longout, "TT:LENGTH", SetReadoutLength);
        PUBLISH_METHOD(longin,  "TT:LENGTH", GetReadoutLength);
        PUBLISH_METHOD(bo, "TT:FREERUN", SetFreeRunning);
        Publish_bi("TT:FREERUN", FreeRunning);

        /* Turn by turn triggering is rather complicated, and needs to occur
         * in two stages.  The idea is that only a single shot of turn by
         * turn data is captured, and then segments of it are read out. 
         *     Capturing a full waveform is done by writing 1 to the ARM
         * record and then waiting for READY to be signalled: this indicates
         * that a waveform has been read into memory. */
        PUBLISH_METHOD(bo, "TT:ARM", SetArm);
        Publish_bi("TT:READY", LongTrigger);
        Interlock.Publish("TT");

        /* Announce our interest in the trigger. */
        RegisterTriggerEvent(*this, PRIORITY_TT);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed. */
    void OnEvent()
    {
        if (Armed || FreeRunning)
        {
            Armed = false;
            /* Capture the full turn-by-turn waveform of the requested
             * length. */
            LongWaveform.Capture();

            /* Let EPICS know that this has updated. */
            LongTrigger.Write(true);

            /* Also bring the short waveforms up to date. */
            ProcessShortWaveform();
        }
    }

    
private:
    TURN_BY_TURN();     // Needed for PUBLISH_METHOD hacks
    
    /* Waveform length control.  This can be dynamically changed through the
     * EPICS interface. */
    bool SetCaptureLength(int Length)
    {
        if (FreeRunning  &&  Length > ShortWaveformLength)
            /* Ensure free running is disabled if length is large. */
            FreeRunning = false;
        LongWaveform.SetLength(Length);
        return true;
    }

    bool GetCaptureLength(int &Length)
    {
        Length = LongWaveform.GetLength();
        return true;
    }

    /* Waveform readout control: both position and length of the waveforms
     * can be controlled. */
    bool SetReadoutOffset(int Offset)
    {
        /* Allow the offset to be set anywhere within the full long waveform,
         * not just within its current length.  It's harmless and friendly to
         * allow this. */
        if (0 <= Offset  &&  Offset < LongWaveformLength)
        {
            /* Minor optimisation, but ProcessShortWaveform() is pretty
             * expensive. */
            if (ShortOffset != Offset)
            {
                ShortOffset = Offset;
                ProcessShortWaveform();
            }
            return true;
        }
        else
        {
            printf("TT:OFFSET %d is out of range\n", Offset);
            return false;
        }
    }
    
    bool SetReadoutLength(int Length)
    {
        if (0 < Length  &&  Length <= ShortWaveformLength)
        {
            bool Process = Length > (int) ShortWaveform.GetLength();
            ShortWaveform.SetLength(Length);
            IqWaveform.SetLength(Length);
            /* Only process the short waveform if it has grown in length.
             * Otherwise there's nothing to do. */
            if (Process)
                ProcessShortWaveform();
            return true;
        }
        else
        {
            printf("TT:LENGTH %d is out of range\n", Length);
            return false;
        }
    }
    
    bool GetReadoutLength(int &Length)
    {
        Length = ShortWaveform.GetLength();
        return true;
    }

    bool GetCapturedLength(int &Length)
    {
        Length = LongWaveform.WorkingLength();
        return true;
    }

    bool SetFreeRunning(bool Enable)
    {
        if (!Enable  ||  (int)LongWaveform.GetLength() <= ShortWaveformLength)
        {
            FreeRunning = Enable;
            return true;
        }
        else
        {
            printf("Can't enable free running at current length\n");
            return false;
        }
    }


    /* Arming is enough to provoke the capture of a full turn-by-turn waveform
     * on the next Libera event.  Arming also sets the LongTrigger into the
     * not-ready state. */
    bool SetArm(bool Arm)
    {
        /* Only do anything on the transition from false to true: this is the
         * true arming action. */
        if (Arm)
        {
            LongTrigger.Write(false);
            Armed = true;
        }
        return true;
    }

    /* This routine updates the short waveform.  This should be called
     * whenever the long waveform has been read, whenever the offset is
     * changed, and whenever the short waveform grows longer (recalculation
     * is pointless when it shrinks!) */
    void ProcessShortWaveform()
    {
        Interlock.Wait();
        
        /* We copy our desired segment from the long waveform and do all the
         * usual processing. */
        ShortWaveform.CaptureFrom(LongWaveform, ShortOffset);
        ShortWaveform.Cordic();
        ShortWaveform.ABCDtoXYQS();

        /* The IQ waveforms are also a copy of a long waveform segment, but
         * completely raw and unprocessed. */
        IqWaveform.CaptureFrom(LongWaveform, ShortOffset);

        /* Let EPICS know there's stuff to read. */
        Interlock.Ready();
    }
    

    const int LongWaveformLength;
    const int ShortWaveformLength;
    
    LIBERA_WAVEFORM LongWaveform;
    LIBERA_WAVEFORM ShortWaveform;
    LIBERA_WAVEFORM IqWaveform;
    
    TRIGGER LongTrigger;
    INTERLOCK Interlock;
    
    /* This flag is set to enable long waveform capture on the next trigger.
     * It will then be reset, ensuring that only one capture occurs per
     * arming request. */
    bool Armed;
    /* This is the offset into the long waveform for which short waveforms
     * will be returned. */
    int ShortOffset;
    /* If this flag is set then waveform capture will occur repeatedly even
     * when not armed.  This flag cannot be set when the waveform capture
     * length is too large to avoid overloading the processor. */
    bool FreeRunning;
};



TURN_BY_TURN * TurnByTurn = NULL;

bool InitialiseTurnByTurn(
    int LongWaveformLength, int ShortWaveformLength)
{
    TurnByTurn = new TURN_BY_TURN(LongWaveformLength, ShortWaveformLength);
    return true;
}
