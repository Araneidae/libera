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

/* Unified PLL controller framework.
 * 
 * The following routines implement a fairly generic PLL controller framework
 * designed to control both the Libera machine and system clocks.  The design
 * is based around a cascaded sequence of controllers of increasing depths
 * with the actual control parameters and reporting abstracted. */

#define _GNU_SOURCE
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <syslog.h>
#include <pthread.h>

#include "driver/libera.h"
#include "test_error.h"
#include "libera_pll.h"

#include "clockPll.h"

#include "controller.h"



#define INITIALISE_HISTORY(array, value) \
    for (unsigned int i = 0; i < sizeof(array) / sizeof(array[0]); i ++) \
        array[i] = value

/* Adds a point to the start of a history array, moving all other entries up
 * by one position.  The first argument is the array: this must be implicitly
 * sized at the point of call, a pointer will not work. */
#define ADD_TO_HISTORY(array, value) \
    memmove(array + 1, array, sizeof(array) - sizeof(array[0])); \
    array[0] = (value)



/* Truncates clock offset to 32 bit value. */

static int clip_to_int(long long value)
{
    if (value < INT_MIN)
        return INT_MIN;
    else if (value > INT_MAX)
        return INT_MAX;
    else
        return value;
}


/* Called whenever synchronisation appears to have been lost (we're quite
 * touchy about holding this flag).  A message is logged if it really has
 * been dropped. */

static void DropSynchronisation(CONTROLLER *Controller, const char * Reason)
{
    if (Controller->Synchronised == SYNC_SYNCHRONISED)
        log_message(LOG_INFO, "%s: Synchronisation lost, %s (%d)",
            Controller->name, Reason, Controller->PhaseError);
    Controller->Synchronised = SYNC_NO_SYNC;
}


/* Captures the next clock interrupt.  We spend most time waiting for the
 * next interrupt, and so here we drop the synchronisation lock. */

static bool GetClock(CONTROLLER *Controller)
{
    TEST_0(pthread_mutex_unlock, &Controller->Interlock);
    Controller->ClockOk = Controller->GetClock(&Controller->Clock);
    TEST_0(pthread_mutex_lock, &Controller->Interlock);
    return Controller->ClockOk;
}


/* Updates the clock settings given a new clock reading. */

static void UpdateClockState(
    CONTROLLER *Controller, bool PhaseLocked, libera_hw_time_t OldClock)
{
    Controller->PhaseLocked = PhaseLocked;
    if (! PhaseLocked)
        DropSynchronisation(Controller, "phase lock lost");
    
    /* The frequency error is determined by comparing the actual clock advance
     * with the expected nominal advance. */
    libera_hw_time_t ClockFrequency = Controller->Clock - OldClock;

    /* The nominal clock advance (assuming correct frequency and perfect phase
     * lock) is determined by the prescale together with any frequency
     * offset. */
    int NominalAdvance =
        Controller->prescale + Controller->frequency_offset;
    /* If the clock is phase locked then the nominal clock must advance by the
     * nominal advance, otherwise the nominal clock simply tracks the actual
     * clock. */
    if (PhaseLocked)
        Controller->NominalClock += NominalAdvance;
    else
        Controller->NominalClock = Controller->Clock;

    /* The phase offset which we report to the driver is simply the difference
     * between the nominal clock and the actual clock; on the other hand, the
     * phase error which is reported and controlled also takes any programmed
     * phase offset into account. */
    long long PhaseOffset = Controller->NominalClock - Controller->Clock;
    Controller->PhaseError =
        clip_to_int(PhaseOffset + Controller->phase_offset);
    Controller->FrequencyError =
        clip_to_int(NominalAdvance - ClockFrequency);

    /* Check whether excessive phase error causes synchronisation to become
     * lost. */
    int PhaseErrorLimit = Controller->Slewing ?
        Controller->max_slew_phase_error :
        Controller->max_normal_phase_error;
    if (abs(Controller->PhaseError) > PhaseErrorLimit)
        DropSynchronisation(Controller, "excessive phase error");

    /* Finally inform the driver of the current phase and clock
     * values. */
    Controller->NotifyDriver(
        ClockFrequency, PhaseOffset, Controller->PhaseLocked);
}


/* Controller synchronisation status: used to manage controller state and
 * status reporting.  The slew acceptance determines how easily the
 * synchronisation flag is lost. */

typedef enum
{
    PHASE_UNLOCKED,     // Not phase locked, don't try to track phase
    PHASE_LOCK_WIDE,    // Phase locked.  Allow wide acceptance slewing
    PHASE_LOCK_NARROW   // Narrow phase lock, set minimum slew acceptance
} PHASE_LOCK;


/* Reads the current clock, if possible, and updates the frequency and phase
 * error calculations. */

static bool UpdateClock(
    CONTROLLER *Controller, bool OpenLoop, PHASE_LOCK PhaseLock)
{
    if (OpenLoop != Controller->OpenLoop)
        return false;
    else
    {
        if (PhaseLock == PHASE_LOCK_NARROW  &&
                Controller->Synchronised == SYNC_SYNCHRONISED)
            /* Once we're synchronised and narrow phase lock is found restore
             * the narrow phase error limit. */
            Controller->Slewing = false;
        
        libera_hw_time_t OldClock = Controller->Clock;
        if (GetClock(Controller))
            /* Note that we only need to update the internal state if we
             * successfully capture the clock: failed clock capture is
             * handled separately in run_get_clock(). */
            UpdateClockState(
                Controller, PhaseLock != PHASE_UNLOCKED, OldClock);
        return Controller->ClockOk;
    }
}



/* This must be called after every call to GetClock() to report the current
 * state of the controller. */

static void ReportState(CONTROLLER *Controller)
{
    /* Compute message to write to log file.  We log gain or loss of clock or
     * phase lock. */
    const char * LogMessage = NULL;
    if (Controller->CurrentStage == 0 && Controller->PreviousStage != 0)
        LogMessage = "Clock lost";
    else if (Controller->CurrentStage != 0 && Controller->PreviousStage == 0)
        LogMessage = "Clock found";
    else if (Controller->PhaseLocked  &&  ! Controller->WasPhaseLocked)
        LogMessage = "Phase locked";
    else if (! Controller->PhaseLocked  &&  Controller->WasPhaseLocked)
        LogMessage = "Phase lock lost";
    if (LogMessage != NULL)
        log_message(LOG_INFO, "%s: %s", Controller->name, LogMessage);

    Controller->ReportAge += 1;
    if (Controller->CurrentStage != Controller->PreviousStage  ||
        Controller->WasSynchronised != Controller->Synchronised  ||
        Controller->ReportAge > Controller->StatusReportInterval)
    {
        WriteStatus("%cs%d %d\n", 
            Controller->status_prefix,
            Controller->CurrentStage,
            Controller->Synchronised);
        Controller->ReportAge = 0;
    }

    if (Controller->Verbose)
        WriteStatus("%cv%d %d %d\n",
            Controller->status_prefix,
            Controller->FrequencyError,
            Controller->PhaseError,
            Controller->Dac);

    /* Update history. */
    Controller->WasPhaseLocked = Controller->PhaseLocked;
    Controller->PreviousStage = Controller->CurrentStage;
    Controller->WasSynchronised = Controller->Synchronised;
}



static void SetDAC(CONTROLLER *Controller, int dac)
{
    /* Ensure DAC setting is in valid range before assigning. */
    if (dac < 0)
        dac = 0;
    else if (dac >= 0xFFFF)
        dac = 0xFFFF;
    Controller->Dac = dac;
    Controller->SetDAC(Controller->Dac);
}



/*****************************************************************************/
/*                                                                           */
/*                       Specific Stage Controllers                          */
/*                                                                           */
/*****************************************************************************/


/* This routine tunes the clock until the correct frequency is found.  As soon
 * as the frequency has settled true is returned together with the current
 * clock position and the corresponding DAC setting.  If the clock is lost
 * during this process false is returned instead. */

int run_FF(CONTROLLER *Controller, const void *Context)
{
    const FF_PARAMS * Params = (const FF_PARAMS *) Context;
    while (UpdateClock(Controller, false, PHASE_UNLOCKED))
    {
        /* Correct the frequency by offsetting the DAC setting in proportion
         * to the frequency error: this amounts to a pure integration
         * controller on frequency error (or, equivalently, a pure
         * proportional controller on phase error). */
        SetDAC(Controller,
            Controller->Dac + Params->FK * Controller->FrequencyError);
        ReportState(Controller);

        /* Return once the target frequency is reached. */
        if (abs(Controller->FrequencyError) <= 1)
            return +1;
    }
    return 0;
}



/* This is the first part of the phase locked loop for the machine clock.
 * This part of the loop is designed to seek the target phase as quickly as
 * possible without worrying too much about long term phase stability.
 *    On successful capture of the desired phase true is returned together
 * with the updated nominal machine time and the current DAC setting.
 *    If phase is lost or if the clock is lost false is returned. */

/* At the end of phase lock-in we end up oscillating around the target DAC
 * setting: when dropping out to the next stage of the loop we take the
 * average of the last 16 points to get a sensible value. */
#define DAC_HISTORY 16

int run_PI(CONTROLLER *Controller, const void *Context)
{
    const PI_PARAMS * Params = (const PI_PARAMS *) Context;
    
    /* Integrated error: we run a simple PI controller. */
    int tI = 0;
    /* Smoothed squared error for lock detection. */
    double var_err = 1e2;
    /* All DAC computations will be offsets from the nominal DAC set on
     * entry. */
    int nominal_dac = Controller->Dac;
    
    /* Initialise the DAC history buffer with our initial DAC reading so that
     * at least we start with something sensible.  However, this should all be
     * swept out by the time we read this. */
    int dac_history[DAC_HISTORY];
    INITIALISE_HISTORY(dac_history, nominal_dac);

    while (UpdateClock(Controller, false, PHASE_LOCK_WIDE))
    {
        tI += Controller->PhaseError;
        int target_dac = nominal_dac +
            Params->KP * Controller->PhaseError +
            Params->KI * tI;
        /* Remember the DAC setting for breakout. */
        ADD_TO_HISTORY(dac_history, target_dac);

        /* If the DAC hits the limits we have a problem.  If we let the
         * integrator continue to run then we end up overcompensating, and 
         * then oscillating for ages afterwards. If, on the other hand, we
         * simply reset the integrater then we can oscillate forever if we
         * bounce off the limits.  Thus here we simply don't integrate this
         * term -- seems to work. */
        if (target_dac <= 0  ||  target_dac >= 0xFFFF)
            tI -= Controller->PhaseError;

        SetDAC(Controller, target_dac);
        ReportState(Controller);
        
        if (abs(Controller->PhaseError) > Params->MaximumPhaseError)
            /* If the phase error grows too large give up trying to hold the
             * locked phase and hand control back to the frequency seeking
             * code. */
            return -1;

        /* Finally check for stable phase lock: once the phase lock is
         * sufficiently stable, we can hand off to the narrow lock filter. */
        var_err =
            Params->IIR * Controller->PhaseError * Controller->PhaseError +
            (1 - Params->IIR) * var_err;
        if (var_err < 2)
        {
            /* Compute the average DAC value that we've been settling around
             * for the last few cycles and assign this as the "best" DAC
             * value for the fine control filter. */
            int NewDac = 0;
            for (int i = 0; i < DAC_HISTORY; i ++)  NewDac += dac_history[i];
            Controller->Dac = NewDac / DAC_HISTORY;
            /* Hand off to the next stage. */
            return +1;
        }
    }
    return 0;
}



/* This runs a general IIR filter.  Locking is abandoned if the error grows
 * too large. */

int run_IIR(CONTROLLER *Controller, const void *Context)
{
    const IIR_PARAMS * Params = (const IIR_PARAMS *) Context;
    
    /* IIR: we have to keep a history of the last N terms and corrections
     * where N is the order of the filter.  Initialise the history to 0, it's
     * the best we can do! */
    float last_error[Params->Order];
    float last_out  [Params->Order];
    INITIALISE_HISTORY(last_error, 0.0);
    INITIALISE_HISTORY(last_out, 0.0);

    int nominal_dac = Controller->Dac;
    while (UpdateClock(Controller, false, PHASE_LOCK_NARROW))
    {
        /* Compute this stage of the filter.  We allow the adding of an offset
         * value to the computed error: adding 0.5 pushes the target across
         * the cloc threshold: this can be used to operate on the metastable
         * transition.
         *     However, for this to work properly we need a slow filter
         * response, as otherwise we move the frequency too much. */
        float adjusted_error = Controller->PhaseError + Params->Dither;
        /* Compute the IIR output from the A and B coefficients and our
         * history. */
        float this_output = Params->Filter[0].B * adjusted_error;
        for (int i = 0; i < Params->Order; i ++)
            this_output +=
                Params->Filter[i+1].B * last_error[i] -
                Params->Filter[i+1].A * last_out[i];
        /* Advance the historical records. */
        ADD_TO_HISTORY(last_error, adjusted_error);
        ADD_TO_HISTORY(last_out,   this_output);

        /* The output is generated as an offset from the nominal DAC entry on
         * entry. */
        SetDAC(Controller, nominal_dac + (int) round(this_output));
        ReportState(Controller);

        /* During normal operation this filter holds the phase strictly with
         * +-1 sample clock.  If the error grows larger than this then hand
         * over to the outer fast filter. */
        if (abs(Controller->PhaseError) > 2)
            /* Phase error too big.  Drop back to the faster filter. */
            return -1;
    }
    return 0;
}



/*****************************************************************************/
/*                                                                           */
/*                          Top Level Controller                             */
/*                                                                           */
/*****************************************************************************/


/* Simply captures the clock. */

static void run_get_clock(CONTROLLER *Controller)
{
    DropSynchronisation(Controller, "clock lost");
    Controller->PhaseLocked = false;
    Controller->CurrentStage = 0;
    do
    {
        /* While the clock is lost notify the driver using sensible defaults.
         * We fake the frequency to the nominal frequency (to avoid confusing
         * the device driver, which will probably crash if we tell it the
         * truth). */
        Controller->NotifyDriver(
            Controller->prescale + Controller->frequency_offset, 0, false);
        ReportState(Controller);
    } while (!GetClock(Controller));
    Controller->NominalClock = Controller->Clock;
}


/* This runs a simple open loop controller: error terms are calculated and
 * reported, but the DAC is never actually updated! */

static void run_open_loop(CONTROLLER *Controller)
{
    int PreviousStage = Controller->CurrentStage;
    /* Advance a stage on entry. */
    Controller->CurrentStage = Controller->StageCount + 1;
    while (Controller->OpenLoop  &&
           UpdateClock(Controller, true,
                Controller->PhaseLocked ? PHASE_LOCK_WIDE : PHASE_UNLOCKED))
        ReportState(Controller);
    /* Drop back a stage on exit. */
    Controller->CurrentStage = PreviousStage;
}


/* Runs the regular stages of controller until either open loop is selected
 * or the clock is lost. */

static void run_stages(CONTROLLER *Controller)
{
    while (Controller->ClockOk  && !Controller->OpenLoop)
    {
        CONTROLLER_STAGE *Stage =
            &Controller->Stages[Controller->CurrentStage - 1];

        /* Run the controller and advance the stage as requested. */
        Controller->CurrentStage += Stage->Action(Controller, Stage->Context);
        if (Controller->CurrentStage < 1)
            Controller->CurrentStage = 1;
        else if (Controller->CurrentStage > Controller->StageCount)
            Controller->CurrentStage = Controller->StageCount;
    } 
}


/* This runs the controller. */

static void* run_controller(void *Context)
{
    CONTROLLER *Controller = (CONTROLLER *) Context;
    
    /* We take a very simple minded approach to interlocking between the
     * command interpreter and the controller threads: all commands are
     * interpreted under the lock, and the controller holds the lock except
     * while it is reading the clock.  This is easy, requires no subtle
     * analysis, and simply works.
     *    So here we start things off by capturing the lock. */
    TEST_0(pthread_mutex_lock, &Controller->Interlock);
    
    while (true)
    {
        /* First try to capture the clock. */
        run_get_clock(Controller);
        Controller->CurrentStage = 1;

        while (Controller->ClockOk)
        {
            if (Controller->OpenLoop)
                run_open_loop(Controller);
            else
                run_stages(Controller);
        }
    }
    return NULL;
}



/*****************************************************************************/
/*                                                                           */
/*                          Command Interpreter                              */
/*                                                                           */
/*****************************************************************************/


/* Adjust the detune frequency.  Of course, changing the detune drops the
 * synchronisation flag. */

static void SetFrequencyOffset(CONTROLLER *Controller, int Offset)
{
    if (Offset != Controller->frequency_offset)
    {
        DropSynchronisation(Controller, "frequency offset changed");
        Controller->frequency_offset = Offset;
    }
}


/* Manages the synchronisation flag.  Use s1 to start tracking
 * synchronisation before generating a trigger, use s2 to confirm successful
 * synchronisation. */

static void SetSynchronisation(CONTROLLER *Controller, int Command)
{
    switch (Command)
    {
        case SYNC_NO_SYNC:
            /* Supported, but not so useful... */
            DropSynchronisation(Controller, "explicitly dropped");
            break;
        case SYNC_TRACKING:
            /* Only allow synchronisation tracking if we're phase
             * locked. */
            if (Controller->PhaseLocked)
            {
                Controller->Synchronised = SYNC_TRACKING;
                Controller->Slewing = true;
            }
            break;
        case SYNC_SYNCHRONISED:
            /* Don't allow a jump from NO_SYNC to SYNCHRONISED:
             * means synchronisation got lost somewhere. */
            if (Controller->Synchronised == SYNC_TRACKING)
            {
                log_message(LOG_INFO,
                    "%s: Synchronised to trigger", Controller->name);
                Controller->Synchronised = SYNC_SYNCHRONISED;
            }
            break;
    }
}


/* Move the phase relative to the synchronised trigger point. */

static void SetPhaseOffset(CONTROLLER *Controller, int PhaseOffset)
{
    /* Setting the phase offset can potentially introduce a massive phase
     * delta.  As this is clearly deliberate, we temporarily open the slewing
     * interval to avoid dropping the synchronisation flag.  Note the fudge
     * factor to cope with trivial overshoot. */
    if (abs(Controller->phase_offset - PhaseOffset) + 10 >
            Controller->max_normal_phase_error)
        Controller->Slewing = true;
    Controller->phase_offset = PhaseOffset;
}


/* Exceptionally dangerous command for directly writing to individual
 * controllers.  Incorrect use of this command WILL crash the clock PLL! */

static void WriteToController(CONTROLLER *Controller, char *Command)
{
    /* The form of a command is
     *      Wf s a v
     * where f is either I for integers or F for floats, s selects the
     * controller stage, a is the index (in longs) to be written, and v is the
     * value to be written (either int or float). */
    /* We're super lazy about error testing.  If the command is wrong, that's
     * just too bad! */
    char * ParsePtr;
    int Stage = strtol(Command + 2, &ParsePtr, 10);
    int Index = strtol(ParsePtr, &ParsePtr, 10);
    int * Target = (int *) Controller->Stages[Stage].Context + Index;
    switch (Command[1])
    {
        case 'I':
        {
            int Value = strtol(ParsePtr, &ParsePtr, 10);
            log_message(LOG_INFO, "WI %d %d %d", Stage, Index, Value);
            *(int *) Target = Value;
            break;
        }
        case 'F':
        {
            float Value = strtof(ParsePtr, &ParsePtr);
            log_message(LOG_INFO, "WI %d %d %g", Stage, Index, Value);
            *(float *) Target = Value;
            break;
        }
        case 'i':
        {
            log_message(LOG_INFO, "i %d %d = %d",
                Stage, Index, *(int *) Target);
            break;
        }
        case 'f':
        {
            log_message(LOG_INFO, "f %d %d = %g",
                Stage, Index, *(float *) Target);
            break;
        }
        default:
            log_message(LOG_ERR, "Invalid Write command: %s", Command);
    }
}


/* Simple command interpreter.
 *
 * The following commands are for normal operation:
 *  o   Detune: adds offset to the managed frequency
 *  p   Phase offset: moves phase relative to synchronisation point
 *  s   Synchronisation flag control
 *  v   Controls verbosity of status reports
 *
 * The following commands are only intended for diagnostic use:
 *  c   Selects open loop control: DAC is only set externally by d command
 *  d   Set DAC value directly if open loop mode selected
 *  i   Status report interval
 */

void ControllerCommand(CONTROLLER *Controller, char *Command)
{
    TEST_0(pthread_mutex_lock, &Controller->Interlock);
    int arg = atoi(Command + 1);
    switch (Command[0])
    {
        case 'o':   SetFrequencyOffset(Controller, arg);        break;
        case 'p':   SetPhaseOffset(Controller, arg);            break;
        case 's':   SetSynchronisation(Controller, arg);        break;
        case 'c':   Controller->OpenLoop = arg;                 break;
        case 'd':   if (Controller->OpenLoop)
                        SetDAC(Controller, arg);                break;
        case 'v':   Controller->Verbose = arg;                  break;
        case 'i':   Controller->StatusReportInterval = arg;     break;

        case 'W':   WriteToController(Controller, Command);     break;
        default:
            log_message(LOG_ERR, "Unknown command \"%s\"", Command);
    }
    TEST_0(pthread_mutex_unlock, &Controller->Interlock);
}



/* Runs the given controller in its own thread after initialising it. */

bool spawn_controller(CONTROLLER *Controller)
{
    TEST_0(pthread_mutex_init, &Controller->Interlock, NULL);

    /* Start the DAC in the middle of its range on startup. */
    Controller->Dac = 0x8000;
    Controller->OpenLoop = false;
    Controller->Verbose = false;
    Controller->StatusReportInterval = 10;
    
    Controller->WasPhaseLocked = false;
    Controller->PreviousStage = 0;
    Controller->ReportAge = 0;
    
    Controller->Synchronised = SYNC_NO_SYNC;
    Controller->WasSynchronised = SYNC_NO_SYNC;
    Controller->Slewing = false;

    /* Sensible initial defaults for first reports. */
    Controller->PhaseError = 0;
    Controller->FrequencyError = 0;

    pthread_t ThreadId;
    return TEST_0(pthread_create, &ThreadId, NULL, run_controller, Controller);
}
