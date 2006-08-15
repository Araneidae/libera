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

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "thread.h"
#include "trigger.h"
#include "hardware.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"

#include "turnByTurn.h"



class TURN_BY_TURN : I_EVENT
{
public:
    TURN_BY_TURN(int LongWaveformLength, int WindowWaveformLength) :
        LongWaveformLength(LongWaveformLength),
        WindowWaveformLength(WindowWaveformLength),
        LongWaveform(LongWaveformLength),
        WindowIq(WindowWaveformLength),
        WindowAbcd(WindowWaveformLength),
        WindowXyqs(WindowWaveformLength),
        LongTrigger(false)
    {
        WindowOffset = 0;
        /* Make the default capture length equal to one window. */
        WindowLength = WindowWaveformLength;
        LongWaveform.SetLength(WindowLength);
        /* Don't trigger until asked to. */
        Armed = false;
        
        /* Publish the PVs associated with Turn by Turn data. */

        /* Two waveforms providing access to the raw I and Q turn by turn
         * data for each button. */
        WindowIq.Publish("TT");
        /* The basic windowed waveform views on the entire turn by turn
         * buffer.  Each of these provides a view of a sub-array of the
         * captured waveform, with offset and length controlled by the OFFSET
         * and LENGTH fields. */
        WindowAbcd.Publish("TT");
        WindowXyqs.Publish("TT");

        /* Control fields for managing capture and readout. */
        PUBLISH_METHOD_OUT(longout, "TT:CAPLEN",
            SetCaptureLength, GetCaptureLength);
        PUBLISH_METHOD_OUT(longout, "TT:OFFSET",
            SetWindowOffset, WindowOffset);
        PUBLISH_METHOD_OUT(longout, "TT:LENGTH",
            SetWindowLength, WindowLength);
        PUBLISH_METHOD_IN(longin,   "TT:CAPTURED", GetCapturedLength);
        Publish_longin("TT:OFFSET", WindowOffset);

        /* Turn by turn triggering is rather complicated, and needs to occur
         * in two stages.  The idea is that only a single shot of turn by
         * turn data is captured, and then segments of it are read out. 
         *     Capturing a full waveform is done by writing 1 to the ARM
         * record and then waiting for READY to be signalled: this indicates
         * that a waveform has been read into memory. */
        PUBLISH_METHOD_OUT(bo, "TT:ARM", SetArm, Armed);
        Publish_bi("TT:READY", LongTrigger);
        Interlock.Publish("TT");

        /* Announce our interest in the trigger. */
        RegisterTriggerEvent(*this, PRIORITY_TT);
    }


    /* This code is called, possibly indirectly, in response to a trigger
     * event to read and process a First Turn waveform.  The waveform is read
     * and all associated values are computed.
     *    We only process if armed. */
    void OnEvent()
    {
        if (Armed)
        {
            Armed = false;
            /* Capture the full turn-by-turn undecimated waveform of the
             * requested length. */
            LongWaveform.Capture();

            /* Also bring the short waveforms up to date.  Do this before
             * updating the long trigger so that the reader knows there is
             * valid data to read. */
            ProcessShortWaveform();

            /* Let EPICS know that this has updated. */
            LongTrigger.Write(true);
        }
    }

    
private:
    TURN_BY_TURN();     // Needed for PUBLISH_METHOD hacks: see ID<>
    
    /* Waveform length control.  This can be dynamically changed through the
     * EPICS interface. */
    bool SetCaptureLength(int Length)
    {
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
    bool SetWindowOffset(int Offset)
    {
        /* Allow the offset to be set anywhere within the full long waveform,
         * not just within its current length.  It's harmless and friendly to
         * allow this. */
        if (0 <= Offset  &&  Offset < LongWaveformLength)
        {
            /* Minor optimisation, but ProcessShortWaveform() is pretty
             * expensive. */
            if (WindowOffset != Offset)
            {
                WindowOffset = Offset;
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
    
    bool SetWindowLength(int Length)
    {
        if (0 < Length  &&  Length <= WindowWaveformLength)
        {
            bool Process = Length > WindowLength;
            WindowLength = Length;
            WindowIq.SetLength(WindowLength);
            WindowAbcd.SetLength(WindowLength);
            WindowXyqs.SetLength(WindowLength);
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
    
    bool GetCapturedLength(int &Length)
    {
        Length = LongWaveform.WorkingLength();
        return true;
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
        WindowIq.CaptureFrom(LongWaveform, WindowOffset);
        WindowAbcd.CaptureCordic(WindowIq);
        WindowXyqs.CaptureConvert(WindowAbcd);

        /* Let EPICS know there's stuff to read. */
        Interlock.Ready();
    }
    

    const int LongWaveformLength;
    const int WindowWaveformLength;

    /* The captured waveforms. */

    /* Long unprocessed waveform as captured. */
    IQ_WAVEFORMS LongWaveform;

    /* Window into the captured waveform: these three blocks of waveforms are
     * all published to EPICS. */
    IQ_WAVEFORMS WindowIq;
    ABCD_WAVEFORMS WindowAbcd;
    XYQS_WAVEFORMS WindowXyqs;

    /* Trigger for long waveform capture and EPICS interlock for updating the
     * window waaveforms. */
    TRIGGER LongTrigger;
    INTERLOCK Interlock;
    
    /* This flag is set to enable long waveform capture on the next trigger.
     * It will then be reset, ensuring that only one capture occurs per
     * arming request. */
    bool Armed;
    /* This is the offset into the long waveform for which short waveforms
     * will be returned. */
    int WindowOffset;
    /* This is the currently selected window length.  It is also maintained
     * as the working length of the three window waveform blocks. */
    int WindowLength;
};



static TURN_BY_TURN * TurnByTurn = NULL;

bool InitialiseTurnByTurn(
    int LongWaveformLength, int WindowWaveformLength)
{
    TurnByTurn = new TURN_BY_TURN(LongWaveformLength, WindowWaveformLength);
    return true;
}
