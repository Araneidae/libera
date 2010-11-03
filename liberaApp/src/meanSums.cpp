/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009  Michael Abbott, Diamond Light Source Ltd.
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


/* Provide support for trigger to trigger sum calculation. */

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
#include "timestamps.h"
#include "slowAcquisition.h"

#include "meanSums.h"


class MEAN_SUMS : I_EVENT
{
public:
    MEAN_SUMS()
    {
        Publish_longin("MS:COUNT", Samples);
        Publish_longin("MS:MEANS", MeanSum);
        Publish_longin("MS:DELTAS", MeanSumDelta);
        Publish_ai("MS:MEANI", MeanCurrent);
        Publish_ai("MS:DELTAI", MeanCurrentDelta);
        Publish_ai("MS:MEANP", MeanPower);

        Interlock.Publish("MS", true);
        Enable.Publish("MS");

        /* Announce our interest in the postmortem event. */
        RegisterTriggerEvent(*this, PRIORITY_MS);

        Samples = 0;
        MeanSum = 0;
        MeanSumDelta = 0;
        MeanCurrent = 0;
        MeanCurrentDelta = 0;
        MeanPower = 0;
    }


private:
    void OnEvent(int Missed)
    {
        if (!Enable.Enabled())
            return;

        Interlock.Wait();

        int LastMeanSum = MeanSum;
        int LastMeanCurrent = MeanCurrent;
        GetTriggeredAverageSum(MeanSum, Samples);
        PowerAndCurrentFromS(MeanSum/4, MeanPower, MeanCurrent);
        MeanSumDelta = MeanSum - LastMeanSum;
        MeanCurrentDelta = MeanCurrent - LastMeanCurrent;

        /* Let EPICS know there's stuff to read. */
        LIBERA_TIMESTAMP Timestamp;
        GetTriggerTimestamp(Timestamp);
        Interlock.Ready(Timestamp);
    }


    /* Published variables. */
    int Samples;        // Samples between last two triggers
    int MeanSum;        // Mean S value between last two triggers
    int MeanSumDelta;   // Delta between sums
    int MeanCurrent;
    int MeanCurrentDelta;
    int MeanPower;

    /* EPICS interlock. */
    INTERLOCK Interlock;
    ENABLE Enable;
};



bool InitialiseMeanSums()
{
    new MEAN_SUMS();
    return true;
}
