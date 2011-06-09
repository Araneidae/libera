/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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

/* IOC startup, command line processing, initialisation and shutdown. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <dbAccess.h>
#include <db_access.h>
#include <caerr.h>
#include <envDefs.h>
#include <iocInit.h>
#include <asDbLib.h>
#include <asTrapWrite.h>

#include "hardware.h"
#include "firstTurn.h"
#include "turnByTurn.h"
#include "freeRun.h"
#include "slowAcquisition.h"
#include "postmortem.h"
#include "device.h"
#include "persistent.h"
#include "publish.h"
#include "thread.h"
#include "trigger.h"
#include "interlock.h"
#include "sensors.h"
#include "conditioning.h"
#include "waveform.h"
#include "booster.h"
#include "meanSums.h"
#include "versions.h"
#include "fastFeedback.h"
#include "events.h"
#include "convert.h"
#include "configure.h"
#include "attenuation.h"
#include "timestamps.h"


/* External declaration of caRepeater thread.  This should really be
 * published by a standard EPICS header file, but for the time being we pick
 * it up like this. */
extern "C" void caRepeaterThread(void *);



/* This variable records the PID file: if successfully written then it will
 * be removed when terminated. */
static const char * PidFileName = NULL;

/* Device name used for $(DEVICE) part of database. */
static const char * DeviceName = NULL;

/* This controls whether an IOC shell is used.  If not the main thread will
 * sleep indefinitely, waiting for the IOC to be killed by a signal. */
static bool RunIocShell = true;

/* If the IOC shell is not running then this semaphore is used to request IOC
 * shutdown. */
static sem_t ShutdownSemaphore;


/* Configuration settable parameters.
 *
 * Note that although defaults have been defined for all of these values, all
 * of these parameters are normally passed in by the runioc script. */

/* Maximum length of long turn by turn buffer. */
static int LongTurnByTurnLength = 196608;       // 12 * default window length
static int TurnByTurnWindowLength = 16384;
/* Free running window length. */
static int FreeRunLength = 2048;
/* Length of 1024 decimated buffer. */
static int DecimatedShortLength = 190;
/* Number of switch cycles to use in SC operation. */
static int ConditioningSwitchCycles = 8;

/* Synchrotron revolution frequency.  Used for labelling decimated data.
 * This default frequency is the Diamond booster frequency. */
static float RevolutionFrequency = 1892629.155;

/* Fundamental ring parameters.  The defaults are for the Diamond storage
 * ring, but these are always overwritten when called from runioc. */
static int Harmonic = 936;              // Bunches per revolution
static int LmtdPrescale = 53382;        // Prescale for lmtd

static int TurnsPerSwitch = 40;

/* Power scaling factors for FT and SA modes. */
static int S0_FT = 0;
static int S0_SA = 0;

/* Location of the persistent state file. */
static const char * StateFileName = NULL;
/* Whether to remount the rootfs when writing the persistent state. */
static bool RemountRootfs = false;

/* NTP monitoring can be turned off at startup. */
static bool MonitorNtp = true;



/* We take any of the four traditional shutdown signals (HUP, INT, QUIT or
 * TERM) as a request to terminate the IOC.  We let the IOC terminate in an
 * orderly manner, so the task is to interrupt whatever the main ioc loop is
 * up to.
 *     Note that this code is in a signal handler, so we must take care to
 * only call async-safe functions.  This rather restricts what we're allowed
 * to do!
 *     Note also that this handler can be called repeatedly on exit (I'm not
 * sure why: perhaps it gets called on each running thread?) so its actions
 * need to be idempotent.  Fortunately close doesn't seem to mind being called
 * repeatedly, and multiple posts to our shutdown semaphore are entirely
 * inconsequential. */

static void AtExit(int signal)
{
    if (RunIocShell)
        /* If the IOC shell is running then closing stdin is sufficient to
         * cause iocsh() to terminate. */
        close(0);
    else
        /* If the IOC shell is not running then the main thread is waiting
         * for our sem_post. */
        sem_post(&ShutdownSemaphore);
}


/* A quiet signal for general use.  We'll bind this to SIGUSR2. */

static void DoNothing(int signal)
{
}


/* Set up basic signal handling environment.  We configure four shutdown
 * signals (HUP, INT, QUIT and TERM) to call AtExit(). */

static bool InitialiseSignals()
{
    struct sigaction AtExitHandler;
    AtExitHandler.sa_handler = AtExit;
    AtExitHandler.sa_flags = 0;
    struct sigaction DoNothingHandler;
    DoNothingHandler.sa_handler = DoNothing;
    DoNothingHandler.sa_flags = 0;
    return
        /* Block all signals during AtExit() signal processing. */
        TEST_IO(sigfillset(&AtExitHandler.sa_mask))  &&
        /* Catch all the usual culprits: HUP, INT, QUIT and TERM. */
        TEST_IO(sigaction(SIGHUP,  &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGINT,  &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGQUIT, &AtExitHandler, NULL))  &&
        TEST_IO(sigaction(SIGTERM, &AtExitHandler, NULL))  &&

        /* Configure SIGUSR2 to do nothing: we can then use this generally
         * without side effects. */
        TEST_IO(sigfillset(&DoNothingHandler.sa_mask))  &&
        TEST_IO(sigaction(SIGUSR2, &DoNothingHandler, NULL));
}


/* Configures non-interactive (daemon mode) operation. */

static void SetNonInteractive()
{
    close(0);
    RunIocShell = false;
    sem_init(&ShutdownSemaphore, 0, 0);
}



/* This routine spawns a caRepeater thread, as recommended by Andrew Johnson
 * (private communication, 2006/12/04).  This means that this IOC has no
 * external EPICS dependencies (otherwise the caRepeater application needs to
 * be run). */

static bool StartCaRepeater()
{
    epicsThreadId caRepeaterId = epicsThreadCreate(
        "CAC-repeater", epicsThreadPriorityLow,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        caRepeaterThread, 0);
    if (caRepeaterId == 0)
        perror("Error starting caRepeater thread");
    return caRepeaterId != 0;
}


/* The Libera driver is started by starting all of its constituent components
 * in turn.  Here is the natural place for these to be defined. */

static bool InitialiseLibera()
{
    return
        /* Initialise signal handling.  This should happen as early as
         * possible, to ensure that all subsequent processing uses this
         * signal state. */
        InitialiseSignals()  &&
        /* First EPICS related activity. */
        StartCaRepeater()  &&
        /* Ensure the trigger interlock mechanism is working.  This needs to
         * happen before any EPICS communication is attempted. */
        InitialiseTriggers()  &&
        /* Version PVs.  This needs to be done before hardware startup, as it
         * can affect the behaviour of hardware. */
        InitialiseVersions()  &&
        /* Initialise the connections to the Libera device.  This also needs
         * to be done early, as this is used by other initialisation code. */
        InitialiseHardware(TurnsPerSwitch)  &&
        /* Get the event receiver up and running.  This spawns background
         * threads for dispatching trigger events. */
        InitialiseEventReceiver()  &&

        /* Initialise the persistent state system early on so that other
         * components can make use of it. */
        InitialisePersistentState(StateFileName, RemountRootfs)  &&
        /* Initialise the signal conditioning hardware interface. */
        InitialiseSignalConditioning(
            Harmonic, TurnsPerSwitch, ConditioningSwitchCycles)  &&
        /* Initialise conversion code.  This needs to be done fairly early as
         * it is used globally. */
        InitialiseConvert()  &&
        /* Initialise attenuation management. */
        InitialiseAttenuation()  &&
        /* Initialise Libera configuration: switches, etc. */
        InitialiseConfigure()  &&
        /* Timestamp and clock management. */
        InitialiseTimestamps()  &&

        /* Now we can initialise the mode specific components. */

        /* Initialise interlock settings. */
        InitialiseInterlock()  &&
        /* First turn processing is designed for transfer path operation. */
        InitialiseFirstTurn(Harmonic, RevolutionFrequency, S0_FT)  &&
        /* Turn by turn is designed for long waveform capture at revolution
         * clock frequencies. */
        InitialiseTurnByTurn(LongTurnByTurnLength, TurnByTurnWindowLength)  &&
        /* Free run also captures turn by turn waveforms, but of a shorter
         * length that can be captured continously. */
        InitialiseFreeRun(FreeRunLength)  &&
        /* Booster operation is designed for viewing the entire booster ramp
         * at reduced resolution. */
        InitialiseBooster(DecimatedShortLength, RevolutionFrequency)  &&
        /* Postmortem operation is only triggered on a postmortem event and
         * captures the last 16K events before the event. */
        InitialisePostmortem()  &&
        /* Slow acquisition returns highly filtered positions at 10Hz. */
        InitialiseSlowAcquisition(S0_SA)  &&
        /* Mean sums, only enabled if FPGA 2 features present. */
        IF_(Version2FpgaPresent, InitialiseMeanSums())  &&
        /* Initialise the fast feedback interface. */
        InitialiseFastFeedback()  &&
        /* Background monitoring stuff: fan, temperature, memory, etcetera. */
        InitialiseSensors(MonitorNtp);
}



/* Shutdown is a little delicate.  We terminate all our threads in an orderly
 * way, but unfortunately we there's not way to synchronise with the EPICS
 * layer: in particular, this means that EPICS threads will continue calling
 * in until _exit() is called.  This means that we don't want to close most
 * resources, so all we really do below is terminate threads. */

static void TerminateLibera()
{
    TerminateEventReceiver();
    TerminateTimestamps();
    TerminateSlowAcquisition();
    TerminateSignalConditioning();
    TerminatePersistentState();
    TerminateSensors();

    /* On orderly shutdown remove the pid file if we created it.  Do this
     * last of all. */
    if (PidFileName != NULL)
        unlink(PidFileName);
}



/* Write the PID of this process to the given file. */

static bool WritePid(const char * FileName)
{
    FILE * output;
    return
        TEST_NULL(output = fopen(FileName, "w"))  &&
        DO_(
            /* Lazy error checking here.  Really should check that there
             * aren't any errors in any of the following. */
            fprintf(output, "%d", getpid());
            fclose(output);
            /* Remember PID filename so we can remove it on exit. */
            PidFileName = FileName;
        );
}


/* This routine parses a configuration setting.  This will be of the form
 *
 *      <key>=<value>
 *
 * where <key> identifies which value is set and <value> is an integer. */

static bool ParseConfigInt(char *optarg)
{
    static const struct
    {
        const char * Name;
        int & Target;
    } Lookup[] = {
        { "TT", LongTurnByTurnLength },
        { "TW", TurnByTurnWindowLength },
        { "FR", FreeRunLength },
        { "BN", DecimatedShortLength },
        { "SC", ConditioningSwitchCycles },
        { "HA", Harmonic },
        { "LP", LmtdPrescale },
        { "NT", TurnsPerSwitch },
        { "S0FT", S0_FT },
        { "S0SA", S0_SA },
    };

    /* Parse the configuration setting into <key>=<integer>. */
    char * eq = strchr(optarg, '=');
    if (eq == NULL)
    {
        printf("Ill formed config definition: \"%s\"\n", optarg);
        return false;
    }
    *eq++ = '\0';
    char * end;
    int Value = strtol(eq, &end, 0);
    if (eq == end  ||  *end != '\0')
    {
        printf("Configuration value not a number: \"%s=%s\"\n", optarg, eq);
        return false;
    }

    /* Figure out who it belongs to! */
    for (size_t i = 0; i < ARRAY_SIZE(Lookup); i ++)
    {
        if (strcmp(optarg, Lookup[i].Name) == 0)
        {
            Lookup[i].Target = Value;
            return true;
        }
    }

    /* Nope, never heard of it. */
    printf("Unknown configuration value \"%s\"\n", optarg);
    return false;
}


/* Parses a floating point number, reports if invalid. */

static bool ParseFloat(const char *optarg, float &Target)
{
    char *end;
    Target = strtod(optarg, &end);
    if (optarg == end  ||  *end != '\0')
    {
        printf("Not a floating point number: \"%s\"\n", optarg);
        return false;
    }
    else
        return true;
}


/* Prints usage message in response to -h option. */

static void Usage(const char *IocName)
{
    printf(
"Usage: %s [-p <pid-file>] <scripts>\n"
"Runs Libera EPICS ioc with an interactive IOC shell after loading and\n"
"running <scripts> as IOC scripts.\n"
"\n"
"Options:\n"
"    -h             Writes out this usage description.\n"
"    -v             Writes version information\n"
"    -p <pid-file>  Writes pid to <pid-file>.\n"
"    -n             Run non-interactively without an IOC shell\n"
"    -c<key>=<val>  Configure run time parameter.  <key> can be:\n"
"       LT      Length of long turn-by-turn buffer\n"
"       TT      Length of short turn-by-turn buffer\n"
"       TW      Length of turn-by-turn readout window\n"
"       DD      Length of /1024 decimated data buffer\n"
"       SC      Number of switch cycles per conditioning round\n"
"       HA      Harmonic: number of bunches per revolution\n"
"       LP      LMTD prescale factor\n"
"       NT      Turns per switch position\n"
"       S0FT    S0 power scaling for FT mode\n"
"       S0SA    S0 power scaling for SA mode\n"
"    -f <f_mc>      Machine revolution frequency\n"
"    -s <file>      Read and record persistent state in <file>\n"
"    -M             Remount rootfs rw while writing persistent state\n"
"    -d <device>    Name of device for database\n"
"    -N             Disable NTP status monitoring\n"
"\n"
"Note: This IOC application should normally be run from within runioc.\n",
        IocName);
}



/* Process any options supported by the ioc.  See Usage() for the options
 * supported.  Parameters argc and argv are updated to point past the
 * options. */

static bool ProcessOptions(int &argc, char ** &argv)
{
    bool Ok = true;
    while (Ok)
    {
        switch (getopt(argc, argv, "+hvp:nc:f:s:Md:N"))
        {
            case 'h':   Usage(argv[0]);                 return false;
            case 'v':   StartupMessage();               return false;
            case 'p':   Ok = WritePid(optarg);          break;
            case 'n':   SetNonInteractive();            break;
            case 'c':   Ok = ParseConfigInt(optarg);    break;
            case 'f':   Ok = ParseFloat(optarg, RevolutionFrequency);  break;
            case 's':   StateFileName = optarg;         break;
            case 'M':   RemountRootfs = true;           break;
            case 'd':   DeviceName = optarg;            break;
            case 'N':   MonitorNtp = false;             break;
            case '?':
            default:
                fprintf(stderr, "Try `%s -h` for usage\n", argv[0]);
                return false;
            case -1:
                /* All arguments read successfuly.  Consume them and return
                 * success. */
                argc -= optind;
                argv += optind;
                return true;
        }
    }
    /* Drop through to here if processing an option fails. */
    return false;
}



/*****************************************************************************/
/*                                                                           */
/*                            IOC Initialisation                             */
/*                                                                           */
/*****************************************************************************/


#define TEST_EPICS(expr) \
    ( { \
        int __status__ = (expr); \
        if (__status__ != 0) \
            printf("%s (%s, %d): %s (%d)\n", #expr, \
                __FILE__, __LINE__, ca_message(__status__), __status__); \
        __status__ == 0; \
    } )


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                            IOC PV put logging                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Alas dbGetField is rather rubbish at formatting floating point numbers, so we
 * do that ourselves, but the rest formats ok. */
static void FormatField(dbAddr *dbaddr, dbr_string_t *value)
{
#define FORMAT(type, format) \
    do { \
        type *raw = (type *) dbaddr->pfield; \
        for (int i = 0; i < length; i ++) \
            snprintf(value[i], sizeof(dbr_string_t), format, raw[i]); \
    } while (0)

    long length = dbaddr->no_elements;
    switch (dbaddr->dbr_field_type)
    {
        case DBR_FLOAT:
            FORMAT(dbr_float_t, "%.7g");
            break;
        case DBR_DOUBLE:
            FORMAT(dbr_double_t, "%.15lg");
            break;
        default:
            dbGetField(dbaddr, DBR_STRING, value, NULL, &length, NULL);
            break;
    }
#undef FORMAT
}

static void PrintValue(dbr_string_t *value, int length)
{
    if (length == 1)
        printf("%s", value[0]);
    else
    {
        printf("[");
        for (int i = 0; i < length; i ++)
        {
            if (i > 0)  printf(", ");
            printf("%s", value[i]);
        }
        printf("]");
    }
}

static void EpicsPvPutHook(asTrapWriteMessage *pmessage, int after)
{
    dbAddr *dbaddr = (dbAddr *) pmessage->serverSpecific;
    long length = dbaddr->no_elements;
    dbr_string_t *value = (dbr_string_t *) calloc(length, sizeof(dbr_string_t));
    FormatField(dbaddr, value);

    if (after)
    {
        /* Log the message after the event. */
        dbr_string_t *old_value = (dbr_string_t *) pmessage->userPvt;
        printf("%s@%s %s.%s ",
            pmessage->userid, pmessage->hostid,
            dbaddr->precord->name, dbaddr->pfldDes->name);
        PrintValue(old_value, length);
        printf(" -> ");
        PrintValue(value, length);
        printf("\n");

        free(old_value);
        free(value);
    }
    else
        /* Just save the old value for logging after. */
        pmessage->userPvt = value;
}

static bool HookLogging(void)
{
    asTrapWriteRegisterListener(EpicsPvPutHook);
    return true;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static bool AddDbParameter(
    char *&Destination, int &Length, const char * Parameter,
    const char * Format, ...)
{
    char FullFormat[128];
    snprintf(FullFormat, sizeof(FullFormat), "%s=%s,", Parameter, Format);

    va_list args;
    va_start(args, Format);
    int Output = vsnprintf(Destination, Length, FullFormat, args);
    va_end(args);

    if (Output < Length)
    {
        Destination += Output;
        Length -= Output;
        return true;
    }
    else
    {
        printf("Macro buffer overrun on %s\n", Parameter);
        return false;
    }
}

static bool LoadDatabases()
{
    char LiberaMacros[1024];
    char * Buffer = LiberaMacros;
    int Length = sizeof(LiberaMacros);

#define DB_(format, name, value) \
    AddDbParameter(Buffer, Length, (name), (format), (value))
#define LOAD_RECORDS_(db_file) \
    TEST_EPICS(dbLoadRecords((char *) db_file, LiberaMacros))

    return
        /* The following list of parameter must match the list of
         * substitution parameters expected by the .db files. */
        DB_("%s", "DEVICE",         DeviceName)  &&
        DB_("%d", "BN_SHORT",       DecimatedShortLength)  &&
        DB_("%d", "BN_LONG",        16 * DecimatedShortLength)  &&
        DB_("%d", "TT_LONG",        LongTurnByTurnLength)  &&
        DB_("%d", "TT_WINDOW",      TurnByTurnWindowLength)  &&
        DB_("%d", "FR_LENGTH",      FreeRunLength)  &&
        DB_("%d", "SC_IQ_LENGTH",   ConditioningIQlength())  &&
        DB_("%d", "ATTEN_COUNT",    MaximumAttenuation() + 1)  &&
        DB_("%d", "FIR_LENGTH",     FA_DecimationFirLength)  &&

        LOAD_RECORDS_("db/libera.db")  &&
        IF_(Version2FpgaPresent, LOAD_RECORDS_("db/libera-2.0.db"))  &&
        IF_(FastFeedbackFeature, LOAD_RECORDS_("db/fastFeedback.db"));

#undef DB_
#undef LOAD_RECORDS_
}

static bool SetPrompt()
{
    if (DeviceName == NULL)
    {
        printf("DEVICE not set!\n");
        return false;
    }

    int Length = strlen(DeviceName) + 64;    // "%s> " + slack
    char Prompt[Length];
    snprintf(Prompt, Length, "%s> ", DeviceName);
    epicsEnvSet("IOCSH_PS1", Prompt);
    return true;
}


extern "C" int ioc_registerRecordDeviceDriver(struct dbBase *pdbbase);

/* This implements the following st.cmd:
 *
 *      dbLoadDatabase("dbd/ioc.dbd",0,0)
 *      ioc_registerRecordDeviceDriver(pdbbase)
 *      dbLoadRecords("db/libera.db", "${LIBERA_MACROS}")
 *      dbLoadRecords("db/fastFeedback.db", "${LIBERA_MACROS}")
 *      epicsEnvSet "IOCSH_PS1" "${DEVICE}> "
 *      iocInit()
 */

static bool StartIOC()
{
    return
        SetPrompt()  &&
        TEST_EPICS(dbLoadDatabase((char *) "dbd/ioc.dbd", NULL, NULL))  &&
        TEST_EPICS(ioc_registerRecordDeviceDriver(pdbbase))  &&
        LoadDatabases()  &&
        TEST_EPICS(asSetFilename("db/access.acf"))  &&
        TEST_EPICS(iocInit())  &&
        HookLogging();
}


/****************************************************************************/
/*                                                                          */


int main(int argc, char *argv[])
{
    /* Configure stdout for line buffered output so log file entries appear
     * immediately. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    /* Consume any option arguments and start the driver. */
    bool Ok =
        ProcessOptions(argc, argv)  &&
        InitialiseLibera()  &&
        StartIOC();

    /* Consume any remaining script arguments by running them through the IOC
     * shell. */
    for (; Ok  &&  argc > 0; argc--)
        Ok = TEST_EPICS(iocsh(*argv++));

    /* Run the entire IOC with a live IOC shell, or just block with the IOC
     * running in the background. */
    if (Ok)
    {
        StartupMessage();
        if (RunIocShell)
            /* Run an interactive shell. */
            Ok = TEST_EPICS(iocsh(NULL));
        else
        {
            /* Wait for the shutdown request. */
            printf("Running in non-interactive mode.  "
                "Kill process %d to close.\n", getpid());
            fflush(stdout);
            sem_wait(&ShutdownSemaphore);
        }
    }

    /* Finally shut down in as tidy a manner as possible.  Note that this
     * routine may be called without the matching InitialiseLibera() routine
     * being called: all the terminate routines need to handle this
     * possibility cleanly. */
    TerminateLibera();
    printf("Ioc terminated normally\n");

#ifdef __ARM_EABI__
    /* There is some unpleasantness happening behind the scenes, almost
     * certainly inside the EPICS library, causing our shutdown to be untidy
     * -- can, for example, get the message:
     *
     *      terminate called without an active exception
     *
     * which then aborts us.  As this message always occurs *after* main()
     * returns it's due to some atexit(3) function, almost certainly a static
     * destructor.  This message comes from
     *
     *      gcc-4.3.2/libstdc++-v3/libsupc++/vterminate.cc
     *
     * To avoid this nonsense, we just pull the plug here: OS cleanup is good
     * enough for us, I'm pretty sure.
     *
     * Unfortunately, oh joy, this is system dependent.  On older systems
     * (possibly linuxthreads based) this causes the IOC to simply lock up on
     * exit, so we need to do a proper exit.  There doesn't seem to be a
     * simple test for NPTL threads, but fortunately NPTL was introduced to
     * Libera at the same time as ARM EABI, so we use this test.
     *
     * However: we need to flush any file output we're interested in! */
    fclose(stdout);
    fclose(stderr);
    _exit(Ok ? 0 : 1);
#endif
}
