/* This file is part of the Libera EPICS Driver,
 * 
 * Copyright (C) 2006-2009 Michael Abbott, Diamond Light Source Ltd.
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

/* Machine clock PLL. */

#define _GNU_SOURCE
#include <stdbool.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "driver/libera.h"
#include "libera_pll.h"

#include "test_error.h"
#include "clockPll.h"
#include "controller.h"

#include "machineClock.h"



/* Default MC precaler: number of machine clocks between MC tick events. */
static unsigned long mc_presc;
/* Default decimation: number of samples per revolution */
static unsigned long ddc_decimation;
/* Default harmonic number: number of bunches per revolution  */
static unsigned long harmonic;




/*****************************************************************************/
/*                                                                           */
/*                          Interface to Machine                             */
/*                                                                           */
/*****************************************************************************/


/* Returns the current absolute machine time and (if Counts is not NULL) the
 * (estimated) number of tick intervals since the last machine time.  When
 * Counts is non-NULL then *MachineTime must record the previous machine time.
 *
 * This mechanism is currently required because the device driver can block
 * for a very long time, causing counts to become lost. */

static bool GetMachineTime(libera_hw_time_t * MachineTime)
{
    /* Read the machine time.  This ioctl will block until the a machine time
     * can be read (100ms), or a timeout occurs, in which case the ioctl will
     * fail and errno is set to EAGAIN. */
    bool mc_ok =
        ioctl(event_fd, LIBERA_EVENT_GET_MC_TRIGGER_10, MachineTime) == 0;
    /* Normally either the ioctl succeeded, or it failed with a timeout:
     * almost certainly because the machine clock trigger isn't connected.
     * We only want to log something if neither of these cases holds. */
    TEST_OK(mc_ok  ||  errno == EAGAIN);
    return mc_ok;
}


static void SetMachineClockDAC(int dac)
{
    TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_DAC_A, dac));
}


static void NotifyMachineClockDriver(
    libera_hw_time_t Frequency, libera_hw_time_t Phase, bool PhaseLocked)
{
    unsigned long fmc_set = (unsigned long) (100 * Frequency);
    unsigned int Locked = PhaseLocked;
    TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_FLMC,  &fmc_set));
    TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_MCPHI, &Phase));
    TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_MCPLL, &Locked));
}



/*****************************************************************************/
/*                                                                           */
/*                             Clock Controller                              */
/*                                                                           */
/*****************************************************************************/

/* These filter coefficients define a second order IIR filter which is used to
 * managed the phase error.  The goal is to keep the phase error low (to
 * within +-1 or 2 sample clocks) with neither excessive excursions in
 * frequency or long term oscillations -- it turns out that designing such a
 * filter is quite tricky.  The coefficients below work for a system with an
 * open loop gain of approximately 0.03.
 *
 * The filter used here has z-transform
 *
 *                 2
 *             B  z  + B  z + B
 *              0       1      2   B(z)
 *      G(z) = ----------------- = ----
 *               (z-1)(z-beta)     A(z)
 *
 * This is part of a feedback loop involving the VCXO and phase measurement
 * mechanism: this is modelled as
 *
 *             alpha
 *      F(z) = -----
 *              z-1
 *
 * That is to say: the VCXO and its phase error can be modelled simply as an
 * integrator with unit delay and gain factor alpha.  If we then use the
 * filter G above to control this system, we get an overall response to noise
 * (which we can model as simply added to the control input to the VCXO) of
 *
 *                F(z)            a A
 *      PHI = ------------ = ------------  (writing a = alpha)
 *            1 + F(z)G(z)   (z-1)A + a B
 *
 * There are several goals to meet when designing the control filter:
 * 
 *  1. the long term DC response (phase drift) must be zero.  This corresponds
 *     to requiring that A(1) = 0
 *
 *  2. the system PHI must be stable.  This corresponds to requiring that all
 *     of the roots of the polynomial
 *    
 *      R(z) = (z-1) A(z) + a B(z)
 *
 *     lie within the unit circle |z| < 1.
 *
 *  3. the system PHI must have a low overall gain without strong peaks in
 *     frequency response.  This corresponds to requring that the roots of
 *     R(z) be small, ie |z| << 1.
 *
 *  4. the properties above should be preserved as alpha varies over a
 *     reasonable range of possible values
 *
 *  5. the impulse response of the filter G should have magnitude no greater
 *     than 1 (this is not affected by alpha, of course).  This is required
 *     to reduce the magnitude of phase oscillations around zero caused by
 *     the fact that phase error is reported in integer units only.
 *
 * (1) is easy: we just write A with a factor of (z-1).  Achieving the rest is
 * not so straightforward.  The coefficients below seem to be a good
 * compromise with almost the simplest possible A (just setting A=(z-1),
 * corresponding to a simple PI loop, make (5) and (3) mutually incompatible),
 * and works satisfactorily of a range of 0.01 < alpha < 0.1. */



/* Frequency seek controller.  The scaling factor is slightly low, but this
 * works well enough. */
static FF_PARAMS FF_params = { .FK = 20 };

/* Coarse PI controller.  Holds the phase strongly, but tends to overcorrect
 * due to the large controller gain. */
static PI_PARAMS PI_params = 
{
    .KP = 20,
    .KI = 9, 
    .IIR = 0.15,
    // Need to allow for large slews of the machine clock during
    // synchronisation: the maximum possible slew is one fast feedback
    // interval: this is around 20000 sample clocks.
    .MaximumPhaseError = 30000
};

/* Slow IIR controller. */
#define BETA 0.8
static IIR_PARAMS IIR_params = 
{
    .Order = 2,
    .Dither = 0.0,
    //            B_0         B_1    A_1          B_2   A_2
    .Filter = { { 0.3, 0 }, { 0.14, -1-BETA }, { -0.41, BETA } }
};


static CONTROLLER MC_Controller =
{
    .frequency_offset = 0,
    .phase_offset = 0,
    .max_normal_phase_error = 100, 
    .max_slew_phase_error = 30000,  // Allow large error during sync slew

    .name = "MC",
    .status_prefix = 'm',

    .GetClock     = GetMachineTime,
    .SetDAC       = SetMachineClockDAC,
    .NotifyDriver = NotifyMachineClockDriver,

    .StageCount = 3,
    .Stages =
    {
        { .Action = run_FF,  .Context = & FF_params },
        { .Action = run_PI,  .Context = & PI_params },
        { .Action = run_IIR, .Context = & IIR_params }
    }
};



/* The phase advance per sample for the intermediate frequency generator is
 * controlled by an ioctl.  The phase advance is in units of 2^32*f_if/f_s,
 * where f_if is the desired intermediate frequency and f_s is the sample clock
 * frequency.
 *     If we write P=prescale, D=decimation, H=bunches per turn and
 * F=frequency offset then the sample clock satisfies:
 *
 *          f_s = (D/H + F/HP) f_rf
 *
 * We normally want to set f_if = f_rf (modulo f_s), in which case the desired
 * intermediate frequency scaling factor (to ensure that the resampled RF
 * frequency is equal to the IF) is
 *                                                HP
 *          N = 2^32 frac(f_rf/f_s) = 2^32 frac ------
 *                                              PD + F
 */

bool SetNcoFrequency(int nco_offset)
{
    /* We need to calculate the fractional part of f_rf/f_s to get the correct
     * value for N.  As the frequency offset F is always quite small (and is
     * guaranteed to be less than frac(H/D)), we can accurately calculate the
     * integer part of HP/(PD+F) as the integer part of H/D. */
    unsigned long nco = (unsigned long) (
        (double)(1ULL << 32) * (
            ((double) harmonic * mc_presc) /
                ((double) MC_Controller.prescale + nco_offset) -
            harmonic / ddc_decimation));
    return TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_NCO, &nco));
}



void MachineClockCommand(char *Command)
{
    ControllerCommand(&MC_Controller, Command);
}


bool InitialiseMachineClock(MC_PARAMETERS *Params)
{
    harmonic = Params->Harmonic;
    mc_presc = Params->Prescale;
    ddc_decimation = Params->Decimation;
    MC_Controller.prescale = Params->Prescale * Params->Decimation;

    unsigned int init_locked = false;    
    return 
        /* Enable machine clock trigger events. */
        TEST_IO(ioctl(event_fd,
            LIBERA_EVENT_ENABLE_MC_TRIG, TRIGGER_BIT(6)))  &&
        /* Program the NCO to the selected machine clock frequency. */
        SetNcoFrequency(0)  &&
        /* Initially report the machine clock as unlocked.*/
        TEST_IO(ioctl(event_fd, LIBERA_EVENT_SET_MCPLL, &init_locked))  &&
        spawn_controller(&MC_Controller);
}
