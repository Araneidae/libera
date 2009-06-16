/* This file is part of the Libera EPICS Driver,
 * 
 * Copyright (C) 2007 Instrumentation Technologies
 * Copyright (C) 2009 Michael Abbott, Diamond Light Source Ltd.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "test_error.h"
#include "healthd.h"



/* Limits on controlled fan speeds: the controller won't attempt to push
 * beyond these limits. */
#define MAX_FAN_SPEED   5700
#define MIN_FAN_SPEED   2500

/* It really doesn't matter hugely how we start, so to simplify things we
 * assume an initial fan speed of 4000 RPM.  The controller will settle
 * quickly enough anyhow.
 *    The one disadvantage of not reading the fan speed at startup is that
 * restarting the health daemon will force the controller to hunt for the
 * right speed again. */
#define INITIAL_FAN_SPEED   4000


/* Macro to set specified variable to defaults below. */
#define SET_DEFAULT(value, class) \
    if (value == -1) \
        value = use_rf_sensor ? class##_RF : class##_MB

/* Default controller parameters when using RF board sensor. */ 
#define TARGET_TEMP_RF          49
#define PANIC_TEMP_RF           75
#define CONTROLLER_KP_RF        160
#define CONTROLLER_KI_RF        100

/* Default controller parameters when using motherboard sensor. */ 
#define TARGET_TEMP_MB          42
#define PANIC_TEMP_MB           65
#define CONTROLLER_KP_MB        40
#define CONTROLLER_KI_MB        40


/*****************************************************************************/
/*                                                                           */
/*                              Parameters                                   */
/*                                                                           */
/*****************************************************************************/

/* Parameters read from the control line. */
static bool daemon_mode = true;
static bool use_rf_sensor = false;
static int loop_interval = 60;
static const char * panic_action = NULL;
/* Panic temperatures used to force reboot (or configured panic action). */
static int max_temperature_MB = PANIC_TEMP_MB;
static int max_temperature_RF = PANIC_TEMP_RF;

/* These paramters are all sensor dependent.  If they are not specified then
 * defaults will be configured. */
static int target_temperature = -1;
static int controller_KP = -1;
static int controller_KI = -1;

/* The health daemon can be externally turned on and off. */
static bool enabled = true;

static int verbosity = 0;


/*****************************************************************************/
/*                                                                           */
/*                             Error Logging                                 */
/*                                                                           */
/*****************************************************************************/

/* Routine for printing an error message complete with associated file name
 * and line number. */
/* This is stolen from liberaApp/src/hardware.cpp and belongs in a shared
 * library. */

void print_error(const char * Message, const char * FileName, int LineNumber)
{
    /* Large enough not to really worry about overflow.  If we do generate a
     * silly message that's too big, then that's just too bad. */
    const int MESSAGE_LENGTH = 512;
    int Error = errno;
    char ErrorMessage[MESSAGE_LENGTH];
    
    int Count = snprintf(ErrorMessage, MESSAGE_LENGTH,
        "%s (%s, %d)", Message, FileName, LineNumber);
    if (errno != 0)
    {
        /* This is very annoying: strerror() is not not necessarily thread
         * safe ... but not for any compelling reason, see:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00101.html
         * and the rather unhelpful reply:
         *  http://sources.redhat.com/ml/glibc-bugs/2005-11/msg00108.html
         *
         * On the other hand, the recommended routine strerror_r() is
         * inconsistently defined -- depending on the precise library and its
         * configuration, it returns either an int or a char*.  Oh dear.
         *
         * Ah well.  We go with the GNU definition, so here is a buffer to
         * maybe use for the message. */
        char StrError[256];
        snprintf(ErrorMessage + Count, MESSAGE_LENGTH - Count,
            ": (%d) %s", Error, strerror_r(Error, StrError, sizeof(StrError)));
    }
    if (daemon_mode)
        syslog(LOG_ERR, "%s", ErrorMessage);
    else
        fprintf(stderr, "%s\n", ErrorMessage);
}


void log_message(int Priority, const char * Format, ...)
    __attribute__((format(printf, 2, 3)));
void log_message(int Priority, const char * Format, ...)
{
    va_list args;
    va_start(args, Format);
    char Message[128];
    vsnprintf(Message, sizeof(Message), Format, args);
    
    if (daemon_mode)
        syslog(Priority, "%s", Message);
    else
        printf("%d: %s\n", Priority, Message);
}



/*****************************************************************************/
/*                                                                           */
/*                           Command and Control                             */
/*                                                                           */
/*****************************************************************************/


/* The signal handler shuts the process down without any ceremony.  We rely
 * on normal OS handling to clean up the residue. */

void ExitHandler(int signo) __attribute__((noreturn));
void ExitHandler(int signo)
{
    /* Make sure we don't leave the PID file and command FIFO behind -- do
     * this last of all. */
    unlink(HEALTHD_PID_FILE);
    unlink(HEALTHD_COMMAND_FIFO);

    log_message(LOG_INFO, "Health daemon exiting");
    
    /* Die NOW! */
    fclose(stdout);
    _exit(0);
}


static bool ParseInt(const char *String, int *result)
{
    char *end;
    *result = strtol(String, &end, 0);
    return end > String  &&  *end == '\0';
}


static bool SetParameter(char Option, const char * Value)
{
    bool Ok = true;
    switch (Option)
    {
        case 'T':   Ok = ParseInt(Value, &target_temperature);  break;
        case 'm':   Ok = ParseInt(Value, &max_temperature_MB);  break;
        case 'e':   Ok = ParseInt(Value, &max_temperature_RF);  break;
        case 't':   Ok = ParseInt(Value, &loop_interval);       break;
        case 'p':   Ok = ParseInt(Value, &controller_KP);       break;
        case 'i':   Ok = ParseInt(Value, &controller_KI);       break;
        case 'v':   Ok = ParseInt(Value, &verbosity);           break;
        case 'E':   use_rf_sensor = true;                       break;
        case 'M':   use_rf_sensor = false;                      break;
        default:
            return false;
    }
    return Ok;
}


static void DispatchCommand(const char * Command)
{
    char * Newline = strchr(Command, '\n');
    if (Newline == NULL)
        log_message(LOG_ERR, "Malformed command \"%s\"", Command);
    else
    {
        *Newline = '\0';
        if (SetParameter(Command[0], Command + 1))
            log_message(LOG_INFO, "Command %s", Command);
        else if (strcmp(Command, "ON") == 0)
        {
            enabled = true;
            log_message(LOG_INFO, "Health daemon turned on");
        }
        else if (strcmp(Command, "OFF") == 0)
        {
            enabled = false;
            log_message(LOG_WARNING, "Health daemon turned ff");
        }
        else
            log_message(LOG_ERR, "Unknown command \"%s\"", Command);
    }
}


/* This checks for an incoming command on the command FIFO. */

static void * RunCommandLoop(void * context)
{
    FILE * CommandPipe;
    while (TEST_NULL(
        CommandPipe = fopen(HEALTHD_COMMAND_FIFO, "r")))
    {
        char Command[80];
        while (fgets(Command, sizeof(Command), CommandPipe) != NULL)
            DispatchCommand(Command);
        fclose(CommandPipe);
    }
    /* Oops.  This really shouldn't have happened. */
    return NULL;
}



static bool InitialiseCommandLoop(void)
{
    unlink(HEALTHD_COMMAND_FIFO);       // In case it's there
    pthread_t ThreadId;
    return
        TEST_IO(mkfifo(HEALTHD_COMMAND_FIFO, 0666))  &&
        TEST_0(pthread_create(&ThreadId, NULL, RunCommandLoop, NULL));
}




/*****************************************************************************/
/*                                                                           */
/*                         Health Daemon Controller                          */
/*                                                                           */
/*****************************************************************************/


static const char *sensor_temp_RF;  // RF board temperature sensor
static const char *sensor_temp_MB;  // Motherboard temperature sensor
static const char *sensor_fan0;     // Front fan set speed
static const char *sensor_fan1;     // Rear fan set speed

static bool UseSys;     // Whether we're using /sys or /proc


static bool ReadTemperature(const char * temp_sensor, int *temp)
{
    FILE * input;
    bool Ok = TEST_NULL(input = fopen(temp_sensor, "r"));
    if (Ok)
    {
        Ok = TEST_OK(fscanf(input, UseSys ? "%d" : "%*d\t%*d\t%d", temp) == 1);
        if (UseSys)
            *temp /= 1000;
        fclose(input);
    }
    return Ok;
}

static bool ReadTemperatures(int *temp_MB, int *temp_RF)
{
    *temp_RF = 0;
    bool Ok = ReadTemperature(sensor_temp_MB, temp_MB);
    if (Ok  &&  use_rf_sensor)
        Ok = ReadTemperature(sensor_temp_RF, temp_RF);
    return Ok;
}


bool WriteDevice(const char * device, const char * format, ...)
    __attribute__((format(printf, 2, 3)));
bool WriteDevice(const char * device, const char * format, ...)
{
    FILE * output;
    bool Ok = TEST_NULL(output = fopen(device, "w"));
    if (Ok)
    {
        va_list args;
        va_start(args, format);
        Ok = TEST_OK(vfprintf(output, format, args) > 0);
        fclose(output);
    }
    return Ok;
}


#define I2C_DEVICE  "/sys/bus/i2c/devices/"
#define PROC_DEVICE "/proc/sys/dev/sensors/"


static bool InitialiseController(void)
{
    UseSys = access("/sys", F_OK) == 0;
    if (UseSys)
    {
        /* The /sys file system exists.  All our sensors live here. */
        sensor_temp_RF = I2C_DEVICE "0-0018/temp1_input";
        sensor_temp_MB = I2C_DEVICE "0-0029/temp1_input";

        /* Depending on the kernel version, the fan speed is either set by
         * setting pwm1_enable to 2 (closed loop) and writing to fan1_target,
         * or in older version by writing to speed. */
        if (access(I2C_DEVICE "0-004b/speed", F_OK) == 0)
        {
            /* Old style device support. */
            sensor_fan0 = I2C_DEVICE "0-004b/speed";
            sensor_fan1 = I2C_DEVICE "0-0048/speed";
        }
        else if (access(I2C_DEVICE "0-004b/fan1_target", F_OK) == 0)
        {
            /* Newer PWM control.  Need to set sensible default speed and
             * switch fans into appropriate mode. */
            WriteDevice(I2C_DEVICE "0-0048/pwm1_enable", "2");
            WriteDevice(I2C_DEVICE "0-004b/pwm1_enable", "2");
            sensor_fan0 = I2C_DEVICE "0-004b/fan1_target";
            sensor_fan1 = I2C_DEVICE "0-0048/fan1_target";
        }
    }
    else
    {
        /* No /sys file system: revert to the older /proc filesystem.  Note
         * that the rf sensor is assumed to be an ADM1023 -- we shouldn't use
         * the MAX1617 if it is present. */
        sensor_temp_RF = PROC_DEVICE "adm1023-i2c-0-18/temp1";
        sensor_temp_MB = PROC_DEVICE "max1617a-i2c-0-29/temp1";
        sensor_fan0    = PROC_DEVICE "max6650-i2c-0-4b/speed";
        sensor_fan1    = PROC_DEVICE "max6650-i2c-0-48/speed";
    }


    /* Assign defaults to all unassigned parameters. */
    SET_DEFAULT(target_temperature, TARGET_TEMP);
    SET_DEFAULT(controller_KP,      CONTROLLER_KP);
    SET_DEFAULT(controller_KI,      CONTROLLER_KI);

    log_message(LOG_INFO,
        "Health daemon started: sensor: %s, target: %d, KP: %d, KI: %d",
        use_rf_sensor ? "RF" : "MB", target_temperature,
        controller_KP, controller_KI);
    if (use_rf_sensor)
        log_message(LOG_INFO,
            "  MB panic temperature %d, RF panic temperature %d",
            max_temperature_MB, max_temperature_RF);
    else
        log_message(LOG_INFO,
            "  MB panic temperature %d", max_temperature_MB);
    log_message(LOG_INFO,
        "  panic action: %s",
        panic_action == NULL ? "not set" : panic_action);
    
    return true;
}


static void PressPanicButton(const char *Reason, int temp_MB, int temp_RF)
{
    log_message(LOG_ERR,
        "healthd panic, %s, MB: (%d, %d), RF: (%d, %d)",
        Reason, temp_MB, max_temperature_MB, temp_RF, max_temperature_RF);
    if (panic_action == NULL)
        log_message(LOG_ERR, "No panic action specified");
    else
    {
        log_message(LOG_ERR, "Invoking command: %s", panic_action);
        TEST_IO(system(panic_action));
    }
    ExitHandler(0);
}


/* We run a very simple PI control loop, setting the fan speeds to regulate
 * the selected temperature sensor. */

static void StepControlLoop(int *integral)
{
    int temp_MB, temp_RF;
    if (ReadTemperatures(&temp_MB, &temp_RF))
    {
        if (temp_MB > max_temperature_MB  ||  temp_RF > max_temperature_RF)
            PressPanicButton("Over temperature", temp_MB, temp_RF);

        int temp = use_rf_sensor ? temp_RF : temp_MB;
        int error = temp - target_temperature;
        *integral += error;
        int new_speed = INITIAL_FAN_SPEED +
            error * controller_KP + *integral * controller_KI;

        if (verbosity > 0)
            log_message(LOG_INFO,
                "temp = %d, error = %d, integral = %d, new_speed = %d",
                temp, error, *integral, new_speed);

        /* Prevent integrator windup when speed reaches its limits. */
        if (new_speed > MAX_FAN_SPEED)
        {
            new_speed = MAX_FAN_SPEED;
            *integral -= error;
        }
        else if (new_speed < MIN_FAN_SPEED)
        {
            new_speed = MIN_FAN_SPEED;
            *integral -= error;
        }

        /* Write the new target fan speed. */
        WriteDevice(sensor_fan0, "%d", new_speed);
        WriteDevice(sensor_fan1, "%d", new_speed);
    }
    else
        PressPanicButton("Unable to read temperature", 0, 0);
}


static void RunControlLoop(void)
{
    int integral = 0;
    while (true)
    {
        if (enabled)
            StepControlLoop(&integral);
        sleep(loop_interval);
    }
}



/*****************************************************************************/
/*                                                                           */
/*                          Health Daemon Startup                            */
/*                                                                           */
/*****************************************************************************/


static void Usage(const char *Name)
{
    printf(
"Usage: %s [options]\n"
"Regulates fan speed to maintain system temperature\n"
"\n"
"Options:\n"
"    -h     Writes out this usage description.\n"
"    -n     Run interactively, not as a daemon\n"
"    -T:    Specify target temperature in degrees\n"
"    -t:    Specify loop interval in seconds (default is 60 seconds)\n"
"    -p:    Specify KP parameter for control loop\n"
"    -i:    Specify KI parameter for control loop\n"
"    -m:    Specify maximum motherboard temperature\n"
"    -e:    Specify maximum RF board temperature\n"
"    -E     Use RF board temperature sensor (default is motherboard)\n"
"    -M     Use motherboard temperature sensor\n"
"    -x:    Specify program to call in event of temperature overflow\n"
"    -v:    Specify verbosity for debug and diagnostics\n",
        Name);
}


static bool ProcessOptions(int argc, char *argv[])
{
    while (true)
    {
        int opt = getopt(argc, argv, "+hnT:t:p:i:m:e:EMx:v:");
        switch (opt)
        {
            case 'h':
                Usage(argv[0]);
                return false;
                break;
                
            case 'n':   daemon_mode = false;    break;
            case 'x':   panic_action = optarg;  break;
                
            case 'T':   case 'm':   case 'e':   case 't':   case 'v':
            case 'p':   case 'i':   case 'E':   case 'M':
                if (!SetParameter(opt, optarg))
                {
                    fprintf(stderr, "Invalid value \"%s\" for option -%c\n",
                        optarg, opt);
                    return false;
                }
                break;
                
            default:
                fprintf(stderr, "Try `%s -h` for help\n", argv[0]);
                return false;

            case -1:
                /* All arguments read successfully, return success. */
                argc -= optind;
                return TEST_OK(argc == 0);
        }
    }
}



/* Intercept the usual signals for killing the process and place a PID file
 * so that we can be killed in an orderly way while running as a daemon. */

bool InitialiseExitHandler(void)
{
    struct sigaction AtExitHandler;
    AtExitHandler.sa_handler = ExitHandler;
    AtExitHandler.sa_flags = 0;
    
    int PidFile = -1;
    char Pid[32];
    bool Ok =
        /* Block all signals during AtExit() signal processing. */
        TEST_IO(sigfillset(&AtExitHandler.sa_mask))  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_IO(sigaction(SIGHUP,  &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGINT,  &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGQUIT, &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGTERM, &AtExitHandler, NULL))  &&
        
        /* Try to create a new PID file.  If it already exists then we'll
         * fail without any further fuss. */
        TEST_IO(PidFile = open(
            HEALTHD_PID_FILE, O_WRONLY | O_CREAT | O_EXCL, 0644));

    if (Ok)
    {
        /* At this point we push ourself into the background if required.
         * This needs to be done after testing for the PID file but before we
         * actually compute the PID, as the daemon() call will change our
         * process id! */
        if(daemon_mode)
            Ok = TEST_IO(daemon(false, false));

        sprintf(Pid, "%d", getpid());
        Ok = Ok  &&  TEST_IO(write(PidFile, Pid, strlen(Pid)));
        close(PidFile);
    }
    return Ok;
}



int main(int argc, char *argv[])
{
    bool Ok = 
        /* Process command line arguments. */
        ProcessOptions(argc, argv)  &&
        /* Sort out shutdown handling and switch into daemon mode. */
        InitialiseExitHandler();
    if (!Ok)
        return 1;

    Ok =
        /* Initialise the controller, check for fan sensors and controls. */
        InitialiseController()  &&
        InitialiseCommandLoop();
        
    /* Finally, if all is well, run the feedback control loop. */
    if (Ok)
        RunControlLoop();
    
    /* If we get up here then forcibly clean up any dangling resources.  Note
     * that this never returns. */
    ExitHandler(0);
}
