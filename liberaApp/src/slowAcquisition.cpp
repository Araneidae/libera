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


/* Implementation of 10Hz "slow acquisition" data. */

#include <stdio.h>
#include <stdlib.h>

#include "drivers.h"
#include "persistent.h"
#include "publish.h"
#include "thread.h"
#include "trigger.h"
#include "hardware.h"
#include "convert.h"
#include "waveform.h"
#include "support.h"
#include "interlock.h"

#include "slowAcquisition.h"



/* Attenuation for sensible signal level at input power of 0dBm, about 45
 * dBm. */
#define A_0                     (45 * DB_SCALE)         // 45 dBm

/* Recorded S level at 45dB attenuation and input power 0dBm. */
#define S_0                     100000000               // 1e8




class SLOW_ACQUISITION : public THREAD
{
public:
    SLOW_ACQUISITION() :
        P_0(to_dB(S_0) + A_0)                   // 20 log_10(S_0) + A_0
    {
        /* Default current scale of 800mA at 0dBm. */
        CurrentScale = 800 * 100000;
        /* Ensure we initialise attenuation stuff! */
        CurrentAttenuation = -1;
        
        Publish_ABCD("SA", ABCD);
        Publish_XYQS("SA", XYQS);
        Publish_XYQS("SA", XYQS_CSPI, "C");
        Publish_ai("SA:POWER", Power);
        Publish_ai("SA:CURRENT", Current);
        PUBLISH_METHOD_OUT(ao, "SA:ISCALE", SetCurrentScale, CurrentScale);
        Persistent("SA:ISCALE", CurrentScale);
        Interlock.Publish("SA");

        /* Initialise the current scale associated values with the initial
         * reading. */
        SetCurrentScale(CurrentScale);
    }

private:
    void Thread()
    {
        StartupOk();

        /* We simply run until asked to stop.  Unfortunately we have no way
         * to interrupt ReadSlowAcquisition(), so we might lock up if that
         * stops responding. */
        while (Running())
        {
            ABCD_ROW NewABCD;
            XYQS_ROW NewXYQS;
            if (ReadSlowAcquisition(NewABCD, NewXYQS))
            {
                Interlock.Wait();
                ABCD = NewABCD;
                ABCDtoXYQS(&ABCD, &XYQS, 1);
                XYQS_CSPI = NewXYQS;
                UpdatePowerAndCurrent();
                Interlock.Ready();
            }
        }
    }


    void UpdateCurrentFactor()
    {
        /* The computation of current is computed as
         *
         *                                       A - A_0
         *                                       -------
         *                                S        20
         *      Current = CurrentScale * --- * 10
         *                               S_0
         *                                           
         * where A is the current attenuation, and S is the reading returned
         * by CSPI (in arbitrary units).  We precompute all of this
         * calculation here except for multiplication by S using the "poor
         * man's floating point" class (PMFP): this is hugely faster than
         * real floating point, and helps ensure we have good precision and
         * dynamic range. */
        CurrentFactor =
            (PMFP(from_dB, CurrentAttenuation - A_0) * CurrentScale) / S_0;
    }

    /* Called to configure the current scale. */
    bool SetCurrentScale(int NewCurrentScale)
    {
        CurrentScale = NewCurrentScale;
        UpdateCurrentFactor();
        return true;
    }


    /* Called to manage the attenuator setting: called each time the
     * attenuator value is needed. */
    void UpdateAttenuation()
    {
        int NewAttenuation = ReadCorrectedAttenuation();
        if (NewAttenuation != CurrentAttenuation)
        {
            CurrentAttenuation = NewAttenuation;
            UpdateCurrentFactor();
        }
    }

    /* Computes power and current from the observed S value, the attenuator
     * setting and the current scaling factor thus:
     * 
     *          P = 20 log S + A - P_0
     *          
     *          I = K_M * K_A * S
     *          
     *                  (A-P_0)/20
     *          K_A = 10
     *
     * Here
     *          P = Power
     *          I = Current
     *          K_M = CurrentScale
     *
     *          A = current corrected attenuator reading
     *          P_0 = fixup offset factor
     *
     * The scaling factor K_M is the beam current for 0dBm input power. */
    void UpdatePowerAndCurrent()
    {
        UpdateAttenuation();
        int S = XYQS.S;

        Power = to_dB(S) + CurrentAttenuation - P_0;
        Current = Denormalise(CurrentFactor * S);

        /* Communicate the latest current reading to the interlock. */
        NotifyInterlockCurrent(Current);
    }
    
    
    INTERLOCK Interlock;
    ABCD_ROW ABCD;
    XYQS_ROW XYQS;
    XYQS_ROW XYQS_CSPI;
    int Power;          // Power in dBm * 1e6
    int Current;        // Current in nA
    int CurrentScale;   // Current in nA at 0dBm input power
    
    /* Current attenuation, used to detect attenuation change. */
    int CurrentAttenuation;
    /* Scaling factor for current calculation: updated whenever either the
     * current scale or the attenuation changes. */
    PMFP CurrentFactor;
    
    /* Precomputed offset for power calculation: P_0 = A_0 + 20 log_10(S_0). */
    const int P_0;
};



static SLOW_ACQUISITION * SlowAcquisition = NULL;

bool InitialiseSlowAcquisition()
{
    SlowAcquisition = new SLOW_ACQUISITION();
    return SlowAcquisition->StartThread();
}


void TerminateSlowAcquisition()
{
    if (SlowAcquisition != NULL)
        SlowAcquisition->Terminate();
}
