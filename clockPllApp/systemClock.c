/* This file is part of the Libera EPICS Driver,
 * 
 * Copyright (C) 2008 Michael Abbott, Diamond Light Source Ltd.
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

#define _GNU_SOURCE
#include <stdbool.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "driver/libera.h"
#include "libera_pll.h"

#include "test_error.h"
#include "clockPll.h"
#include "controller.h"

#include "systemClock.h"




static bool GetSystemTime(libera_hw_time_t * SystemTime)
{
    /* Read the system time.  This ioctl will block until the a system time
     * can be read (100ms), or a timeout occurs, in which case the ioctl will
     * fail and errno is set to EAGAIN. */
    bool mc_ok =
        ioctl(event_fd, LIBERA_EVENT_GET_SC_TRIGGER_9, SystemTime) == 0;
    /* Normally either the ioctl succeeded, or it failed with a timeout:
     * almost certainly because the machine clock trigger isn't connected.
     * We only want to log something if neither of these cases holds. */
    TEST_OK(mc_ok  ||  errno == EAGAIN);
    return mc_ok;
}


static void SetSystemClockDAC(int dac)
{
    TEST_(ioctl, event_fd, LIBERA_EVENT_SET_DAC_B, dac);
}


static void NotifySystemClockDriver(
    libera_hw_time_t Frequency, libera_hw_time_t Phase, bool PhaseLocked)
{
    unsigned int Locked = PhaseLocked;
    TEST_(ioctl, event_fd, LIBERA_EVENT_SET_SCPHI, &Phase);
    TEST_(ioctl, event_fd, LIBERA_EVENT_SET_SCPLL, &Locked);
}




/* Frequency seek controller.  The scaling factor is slightly low, but this
 * works well enough. */
static FF_PARAMS FF_params = { .FK = 15 };

/* Coarse PI controller.  Holds the phase strongly, but tends to overcorrect
 * due to the large controller gain. */
static PI_PARAMS PI_params = 
{
    .KP = 12,
    .KI = 5, 
    .IIR = 0.15, 
    .MaximumPhaseError = 100    // Slewing the system clock doesn't happen
};

/* Slow IIR controller. */
#define BETA 0.8
static IIR_PARAMS IIR_params = 
{
    .Order = 2,
    .Dither = 0.0,
    //            B_0          B_1    A_1          B_2   A_2
    .Filter = { { 0.15, 0 }, { 0.07, -1-BETA }, { -0.205, BETA } }
};


static CONTROLLER SC_Controller =
{
    .prescale = 12500000,   // 125 MHz reference clock sampled at 10Hz
    .frequency_offset = 0,
    .phase_offset = 0,
    .max_normal_phase_error = 10,
    .max_slew_phase_error = 10,

    .name = "SC",
    .status_prefix = 's',

    .GetClock     = GetSystemTime,
    .SetDAC       = SetSystemClockDAC,
    .NotifyDriver = NotifySystemClockDriver,

    .StageCount = 3,
    .Stages =
    {
        { .Action = run_FF,  .Context = & FF_params },
        { .Action = run_PI,  .Context = & PI_params },
        { .Action = run_IIR, .Context = & IIR_params }
    }
};



static void* SystemClockThread(void*Context)
{
    run_controller(&SC_Controller);
    return NULL;
}


void SystemClockCommand(char *Command)
{
    ControllerCommand(&SC_Controller, Command);
}


bool InitialiseSystemClock()
{
    unsigned int init_locked = false;
    pthread_t ThreadId;
    return 
        /* Enable machine clock trigger events. */
        TEST_(ioctl, event_fd, LIBERA_EVENT_ENABLE_SC_TRIG, TRIGGER_BIT(5))  &&
        /* Initially report the machine clock as unlocked.*/
        TEST_(ioctl, event_fd, LIBERA_EVENT_SET_SCPLL, &init_locked)  &&
        TEST_0(pthread_create, &ThreadId, NULL, SystemClockThread, NULL);
}
