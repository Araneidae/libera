/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
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
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "events.h"
#include "convert.h"
#include "waveform.h"
#include "versions.h"

#include "postmortem.h"

#define POSTMORTEM_LENGTH       16384


static void SetPmTrigger();
static bool SetTriggerSource(int NewSource);
static void SetTriggerMode();


class POSTMORTEM : I_EVENT
{
public:
    POSTMORTEM() :
        WaveformIq(POSTMORTEM_LENGTH),
        WaveformAbcd(POSTMORTEM_LENGTH),
        WaveformXyqs(POSTMORTEM_LENGTH),
        Flags(POSTMORTEM_LENGTH)
    {
        /* Publish all the waveforms and the interlock. */
        WaveformIq.Publish("PM");
        WaveformAbcd.Publish("PM");
        WaveformXyqs.Publish("PM");

        /* Publish the interlock flags. */
        Publish_waveform("PM:FLAGS", Flags);
        Publish_longin("PM:X_OFFSET", X_offset);
        Publish_longin("PM:Y_OFFSET", Y_offset);
        Publish_longin("PM:ADC_OFFSET", ADC_offset);
        Publish_bi("PM:X_OFL", X_overflow);
        Publish_bi("PM:Y_OFL", Y_overflow);
        Publish_bi("PM:ADC_OFL", ADC_overflow);

        /* Publish the retriggering controls. */
        OneShotTrigger = false;
        CanRetrigger = true;
        PUBLISH_CONFIGURATION(bo, "PM:MODE", OneShotTrigger, SetTriggerMode);
        Publish_bi("PM:READY", CanRetrigger);
        PUBLISH_METHOD_ACTION("PM:REARM", RearmTrigger);

        /* Finally publish all the PM trigger source controls. */
        TriggerSource = 0;          // Hardware trigger source by default
        MinX = MinY = -1000000;     // Plausible initial defaults
        MaxX = MaxY = 1000000;
        OverflowLimit = 30000;
        OverflowTime = 5;
        PUBLISH_CONFIGURATION(ao,      "PM:MINX", MinX,          SetPmTrigger);
        PUBLISH_CONFIGURATION(ao,      "PM:MAXX", MaxX,          SetPmTrigger);
        PUBLISH_CONFIGURATION(ao,      "PM:MINY", MinY,          SetPmTrigger);
        PUBLISH_CONFIGURATION(ao,      "PM:MAXY", MaxY,          SetPmTrigger);
        PUBLISH_CONFIGURATION(longout, "PM:OVER", OverflowLimit, SetPmTrigger);
        PUBLISH_CONFIGURATION(longout, "PM:TIME", OverflowTime , SetPmTrigger);

#ifdef __EBPP_H_2
        TriggerOffset = 0;
        PUBLISH_CONFIGURATION(longout, "PM:OFFSET",TriggerOffset, SetPmOffset);
#endif

        PUBLISH_CONFIGURATION(
            mbbo, "PM:SOURCE", TriggerSource, SetTriggerSource);
        RealSetTriggerSource(TriggerSource);

        /* Announce our interest in the postmortem event. */
        Interlock.Publish("PM", true);
        RegisterPostmortemEvent(*this, PRIORITY_PM);
    }

    void RealSetPmTrigger()
    {
        if (Version2FpgaPresent)
            WritePmTriggerParameters(
                (PM_TRIGGER_SOURCE) TriggerSource,
                MinX, MaxX, MinY, MaxY, OverflowLimit, OverflowTime);
    }

    bool RealSetTriggerSource(int NewSource)
    {
        bool Ok = Version2FpgaPresent || NewSource == PM_SOURCE_HARDWARE;
        TriggerSource = Ok ? NewSource : PM_SOURCE_HARDWARE;
        RealSetPmTrigger();
        return Ok;
    }

    void RealSetTriggerMode()
    {
        CanRetrigger = true;
    }


private:
    bool RearmTrigger()
    {
        CanRetrigger = true;
        return true;
    }

    void OnEvent(int Missed)
    {
        /* We could log missed triggers here, but that's not such a good idea,
         * as the log file tends to fill up!
        if (Missed > 0)
            printf("%d PM trigger(s) missed\n", Missed);
         */

        /* If single shot triggering selected and we've had our single shot,
         * just silently ignore this trigger. */
        if (!CanRetrigger)
            return;

        /* Wait for EPICS to be ready. */
        Interlock.Wait();

        /* Capture and convert everything. */
        WaveformIq.CapturePostmortem();
        WaveformAbcd.CaptureCordic(WaveformIq);
        WaveformXyqs.CaptureConvert(WaveformAbcd);

        /* Process the interlock event flags. */
        ProcessFlags();
        CanRetrigger = ! OneShotTrigger;

        /* Let EPICS know there's stuff to read. */
        Interlock.Ready(WaveformIq.GetTimestamp());
    }

    /* Processes the interlock and switch event flags in the bottom bit of
     * each work.  The eight bits are aggregated into the Flags waveform and
     * three bits are used to compute X, Y and ADC offsets and overflow.
     *     The following flags are handled specially:
     *          AQ => ADC overflow
     *          BI => X overflow
     *          BQ => Y overflow
     */
    void ProcessFlags()
    {
        LIBERA_ROW * Row = (LIBERA_ROW *) WaveformIq.Waveform();
        unsigned char * flags = Flags.Array();
        /* Prepare the flags. */
        for (int i = 0; i < POSTMORTEM_LENGTH; i++)
        {
            unsigned char flag = 0;
            for (int j = 0; j < 2*BUTTON_COUNT; j ++)
                flag |= (Row[i][j] & 1) << j;
            flags[i] = flag;
        }

        /* Extract the offset and overflow marks. */
        FindOverflow(1, ADC_offset, ADC_overflow);
        FindOverflow(2, X_offset,   X_overflow);
        FindOverflow(3, Y_offset,   Y_overflow);
    }

    void FindOverflow(int bit, int &offset, bool &overflow)
    {
        unsigned char * flags = Flags.Array();
        int mask = 1 << bit;
        overflow = false;
        offset = 0;
        for (int i = 0; i < POSTMORTEM_LENGTH; i++)
        {
            if (flags[i] & mask)
            {
                overflow = true;
                return;
            }
            else
                offset += 1;
        }
    }


    /* Captured and processed waveforms: these three blocks of waveforms are
     * all published to EPICS. */
    IQ_WAVEFORMS WaveformIq;
    ABCD_WAVEFORMS WaveformAbcd;
    XYQS_WAVEFORMS WaveformXyqs;

    /* Interlock overflow flags. */
    UCHAR_WAVEFORM Flags;
    int X_offset, Y_offset, ADC_offset;
    bool X_overflow, Y_overflow, ADC_overflow;

    /* Retriggering control. */
    bool OneShotTrigger;
    bool CanRetrigger;

    /* Postmortem trigger source control. */
    int TriggerSource;
    int MinX, MaxX, MinY, MaxY;
    int OverflowLimit;
    int OverflowTime;

#ifdef __EBPP_H_2
    int TriggerOffset;
#endif

    /* EPICS interlock. */
    INTERLOCK Interlock;
};



static POSTMORTEM * Postmortem = NULL;

static void SetPmTrigger()   { Postmortem->RealSetPmTrigger(); }
static void SetTriggerMode() { Postmortem->RealSetTriggerMode(); }
static bool SetTriggerSource(int NewSource)
    { return Postmortem->RealSetTriggerSource(NewSource); }


bool InitialisePostmortem()
{
    Postmortem = new POSTMORTEM();
    return true;
}
