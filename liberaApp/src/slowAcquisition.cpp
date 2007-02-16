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

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "convert.h"
#include "waveform.h"
#include "numeric.h"
#include "interlock.h"

#include "slowAcquisition.h"



/* Recorded S level at 45dB attenuation and input power 0dBm.  Used to scale
 * absolute power and current computations. */
#define S_0                     100000000               // 1e8



class SLOW_ACQUISITION : public THREAD
{
public:
    SLOW_ACQUISITION() :
        THREAD("SLOW_ACQUISITION"),
        P_0(to_dB(S_0) + A_0),                  // 20 log_10(S_0) + A_0
        S_0_INV(Reciprocal(S_0))                // 1 / S_0
    {
        Publish_ABCD("SA", ABCD);
        Publish_XYQS("SA", XYQS);
        Publish_XYQS("SA", XYQS_CSPI, "C");
        Publish_ai("SA:POWER", Power);
        Publish_ai("SA:CURRENT", Current);
        Interlock.Publish("SA");
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

    /* Suppress use of pthread_cancel in this thread: it can cause trouble! */
    void OnTerminate() { }


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
        Power = to_dB(XYQS.S) + ReadCorrectedAttenuation() - P_0;
        Current = ComputeScaledCurrent(S_0_INV, XYQS.S);

        /* Communicate the latest current reading to the machine protection
         * interlock. */
        NotifyInterlockCurrent(Current);
    }
    
    
    INTERLOCK Interlock;
    ABCD_ROW ABCD;
    XYQS_ROW XYQS;
    XYQS_ROW XYQS_CSPI;
    int Power;          // Power in dBm * 1e6
    int Current;        // Current in 10*nA
    
    
    /* Precomputed offset for power calculation: P_0 = A_0 + 20 log_10(S_0). */
    const int P_0;
    /* Precomputed scaling factor for current calculation, S_0_INV = 1/S_0. */
    const PMFP S_0_INV;
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
