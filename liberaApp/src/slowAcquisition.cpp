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


/* Implementation of 10Hz "slow acquisition" data. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "hardware.h"
#include "thread.h"
#include "trigger.h"
#include "convert.h"
#include "configure.h"
#include "waveform.h"
#include "numeric.h"
#include "interlock.h"

#include "slowAcquisition.h"



/* Recorded S level at 45dB attenuation and input power 0dBm.  Used to scale
 * absolute power and current computations. */
static int S_0 = 0;

static int P_0;
static PMFP S_0_INV;

static void InitialisePowerAndCurrent(int S0_SA)
{
    S_0 = S0_SA;                // 20 log_10(S_0) + A_0
    P_0 = to_dB(S_0) + A_0;     // 1 / S_0
    S_0_INV = Reciprocal(S_0);
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
void PowerAndCurrentFromS(int S, int &Power, int &Current)
{
    Power = to_dB(S) + ReadCorrectedAttenuation() - P_0;
    Current = ComputeScaledCurrent(S_0_INV, S);
}



class SLOW_ACQUISITION : public THREAD
{
public:
    SLOW_ACQUISITION() :
        THREAD("SLOW_ACQUISITION")
    {
        Publish_ABCD("SA", ABCD);
        Publish_XYQS("SA", XYQS);
        Publish_ai("SA:POWER", Power);
        Publish_ai("SA:CURRENT", Current);
        Publish_longin("SA:MAXADC", MaxAdc);
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
                PowerAndCurrentFromS(XYQS.S, Power, Current);
                NotifyInterlockCurrent(Current);
                MaxAdc = ReadMaxAdc();
                Interlock.Ready();
            }
        }
    }


#ifdef UNSAFE_PTHREAD_CANCEL
    /* Suppress use of pthread_cancel in this thread: it can cause trouble! */
    void OnTerminate() { }
#endif


    INTERLOCK Interlock;
    ABCD_ROW ABCD;
    XYQS_ROW XYQS;
    int Power;          // Power in dBm * 1e6
    int Current;        // Current in 10*nA
    int MaxAdc;         // Raw MaxADC reading
};



static SLOW_ACQUISITION * SlowAcquisition = NULL;

bool InitialiseSlowAcquisition(int S0_SA)
{
    InitialisePowerAndCurrent(S0_SA);
    SlowAcquisition = new SLOW_ACQUISITION();
    return SlowAcquisition->StartThread();
}


void TerminateSlowAcquisition()
{
    if (SlowAcquisition != NULL)
        SlowAcquisition->Terminate();
}
