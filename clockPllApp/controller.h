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


struct CONTROLLER;


/* A PLL controller function is passed both its controlling controller and
 * its associated parameters, together with the last clock value read (which
 * can be assumed to be stabilised, in other words with zero error) together
 * with the current dac setting.
 *    The controller should return +1 to advance to the next stage, -1 to
 * return to an earlier stage, or 0 if UpdateClock() returned false. */
typedef int CONTROLLER_ACTION(
    struct CONTROLLER *Controller, const void * Context);

typedef struct
{
    CONTROLLER_ACTION *Action;
    const void * Context;
} CONTROLLER_STAGE;


/* This structure defines the parameters required to run an abstract PLL
 * controller. */
typedef struct CONTROLLER
{
    /*************************************************************************/
    /* The following parameters must be specified to define the controller.  */
    
    /* Clock definition parameters. */
    unsigned int prescale;      // Nominal phase advance per tick
    int frequency_offset;       // Extra phase advance (frequency shift)
    int phase_offset;           // Phase offset relative to synchronisation
    int max_normal_phase_error; // Maximum phase allowed when synchronised
    int max_slew_phase_error;   // Maximum phase allowed during slewing

    const char * name;          // Name of controller for logging
    char status_prefix;         // Prefix character used for reporting

    /* Returns true and the current clock reading, or returns false if the
     * clock cannot be read. */
    bool (*GetClock)(libera_hw_time_t *clock);
    /* Sets the DAC to the given value: this will be clipped to the range
     * 0..65535. */
    void (*SetDAC)(int dac);
    /* Reports the current phase and frequency readings to the driver. */
    void (*NotifyDriver)(
        libera_hw_time_t Frequency, libera_hw_time_t Phase, bool PhaseLocked);

    
    /*************************************************************************/
    /* The following variables are private to the controller.                */
    
    /* Controller state control. */
    bool ClockOk;               // True iff last reading of clock was ok
    bool OpenLoop;              // Controls open loop behaviour
    bool PhaseLocked;           // True iff currently phase locked
    int Dac;                    // Current DAC setting
    libera_hw_time_t Clock;     // Current clock position
    libera_hw_time_t NominalClock;  // Nominal clock position if phase locked
    int PhaseError;             // Current phase error
    int FrequencyError;         // Current frequency error
    int CurrentStage;           // Index of currently active stage
    bool Slewing;               // True during programmed slewing

    /* Status reporting control. */
    bool Verbose;               // Enables error and DAC reporting
    int StatusReportInterval;   // How often status is reported
    bool WasPhaseLocked;
    int PreviousStage;
    int ReportAge;

    /* Synchronisation holding. */
    PLL_SYNC_STATE Synchronised;    // Synchronisation state
    PLL_SYNC_STATE WasSynchronised;

    pthread_mutex_t Interlock;  // Interlock between commands and control

    /* End of controller private variables.                                  */
    

    /*************************************************************************/
    /* The following parameters must be specified to define the controller.  */

    /* Array of controller stages.  (This has to come last because of the open
     * ended list.) */
    int StageCount;
    CONTROLLER_STAGE Stages[];
} CONTROLLER;



/* Types of controllers and their control parameters.  The controller
 * framework above allows extra controller actions to be defined. */

/* Frequency seeker.  Simply seeks the target frequency. */
typedef struct
{
    int FK;
} FF_PARAMS;
CONTROLLER_ACTION run_FF;

/* Simple PI controller. */
typedef struct
{
    int KP;
    int KI;
    float IIR;
    int MaximumPhaseError;
} PI_PARAMS;
CONTROLLER_ACTION run_PI;

/* More general IIR filter. */
typedef struct
{
    int Order;      // Number of poles in the filter
    float Dither;   // Can set to 0.5 for dithered error
    /* The array of filter coefficients should have Order+1 entries with
     * coefficients corresponding to an IIR filter of the form:
     *
     *  y  = B  x  + B  x    + ... + B  x    - A  y    - ... - A  y
     *   n    0  n    1  n-1          N  n-1    1  n-1          N  n-N
     *
     * where B_i = Filter[i].B, A_i = Filter[i].A, x_n is the input at time n
     * and y_n is the filter output at time n and N = Order.  Note that A_0
     * is not used.
     *
     * The z transform of this filter is
     *
     *                 -1            -N      N      N-1
     *         B  + B z   + ... + B z     B z  + B z   + ... + B
     *          0    1             N       0      1             N
     *  F(z) = ------------------------ = ----------------------- ,
     *                 -1            -N     N      N-1      
     *          1 + A z   + ... + A z      z  + A z   + ... + A 
     *               1             N             1             N
     * 
     * hence the description as an N-pole filter. */
    struct { float B, A; } Filter[];
} IIR_PARAMS;
CONTROLLER_ACTION run_IIR;


/* Controller. */
bool spawn_controller(CONTROLLER *Controller);

/* Command interpreter for controller. */
void ControllerCommand(CONTROLLER *Controller, char *Command);
