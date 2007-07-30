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


#define EBPP
#include "cspi.h"

/* Exports for libera device driver interface routines. */



/*****************************************************************************/
/*                                                                           */
/*                             Data Structures                               */
/*                                                                           */
/*****************************************************************************/

#define BUTTON_COUNT    4


/* A row of "data on demand" Libera data consists of eight integers
 * containing quadrature data (cos/sin pairs) for each of the four buttons
 * thus: 
 *      0, 1    A * (cos,sin)
 *      2, 3    B * (cos,sin)
 *      4, 5    C * (cos,sin)
 *      6, 7    D * (cos,sin)
 * Each (cos,sin) pair must be converted to magnitude before further
 * processing.
 *     This data can be read at machine revolution clock frequency, or
 * decimated to 1/64th. */
typedef int LIBERA_ROW[2*BUTTON_COUNT];

/* Raw IQ data.  This is identical in layout to the LIBERA_ROW structure. */
struct IQ_ROW
{
    int AI, AQ;
    int BI, BQ;
    int CI, CQ;
    int DI, DQ;
};

/* Button values. */
struct ABCD_ROW
{
    int A, B, C, D;
};

/* Computed X, Y values in nm, S in arbitrary units. */
struct XYQS_ROW
{
    int X, Y, Q, S;
};


/* The ADC data is read directly from the ADC converter at the sample rate of
 * 117MHz.  Each row consists of four 12-bit signed values (not sign extended
 * in the current release of the driver), one for each button.  ADC data is
 * always read in 1024 row segments. */
#define ADC_LENGTH      1024
typedef short ADC_ROW[BUTTON_COUNT];
typedef ADC_ROW ADC_DATA[ADC_LENGTH];




/*****************************************************************************/
/*                                                                           */
/*                       Hardware Interface Routines                         */
/*                                                                           */
/*****************************************************************************/

/* To be called once on startup to initialise connection to Libera device.
 * If this routine fails (and returns false) then no further operations can
 * be done and system startup should fail. */
bool InitialiseHardware();


#ifdef RAW_REGISTER
/* Writes or writes directly to or from a hardware register.  Not designed for
 * frequent use, as the associated memory mapping is created and deleted each
 * time this routine is called! */
bool WriteRawRegister(unsigned int Address, unsigned int Value);
bool ReadRawRegister(unsigned int Address, unsigned int &Value);
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                   Reading waveform data from the FPGA.                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Reads a waveform of the given length (number of rows) using the given
 * decimation into the given block of data.  Returns the number of rows
 * actually read.
 *     At present only decimation values of 1 or 64 are suppported. */
size_t ReadWaveform(
    int Decimation, size_t WaveformLength, LIBERA_ROW * Data,
    CSPI_TIMESTAMP & Timestamp);

/* Reads the postmortem buffer. */
size_t ReadPostmortem(
    size_t WaveformLength, LIBERA_ROW * Data, CSPI_TIMESTAMP & Timestamp);

/* Reads a full 1024 point ADC waveform. */
bool ReadAdcWaveform(ADC_DATA &Data);

/* Reads a slow acquisition update. */
bool ReadSlowAcquisition(ABCD_ROW &ButtonData, XYQS_ROW &PositionData);

/* Reads (and resets) the maximum ADC reading.  (If the register cannot be
 * read then zero is returned.) */
int ReadMaxAdc();


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      Configuration Setting Routines.                      */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* Interlock settings.   As the CSPI interface requires that we set all the
 * parameters together, we present the interface in one piece. */
bool WriteInterlockParameters(
    CSPI_ILKMODE mode,
    // Valid X,Y positions for interlock
    int Xlow, int Xhigh, int Ylow, int Yhigh,
    // Interlock overflow limit and duration (ADC value and clocks)
    int overflow_limit, int overflow_dur,
    // Gain limit (dBm) for gain-dependent interlock.
    int gain_limit);


/* Sets calibration paramters for calculating (X,Y) from (A,B,C,D). */
bool WriteCalibrationSettings(
    int Kx, int Ky, int Kq, int Xoffset, int Yoffset);



/* Set clock synchronisation. */
bool SetMachineClockTime();
bool SetSystemClockTime(struct timespec & NewTime);
bool GetClockState(bool &LmtdLocked, bool &LstdLocked);


/* Health interrogation.  Returns cspi_health_t directly. */
bool ReadHealth(cspi_health_t &Health);


/* Links the given CSPI event handler to the CSPI event system.  By default
 * this will be delivered through the LIBERA_SIGNAL signal. */
bool ConfigureEventCallback(
    int EventMask, int (*Handler)(CSPI_EVENT*), void *Context);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      DSC Device Interface Routines.                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* There are sixteen possible switch positions. */
#define SWITCH_COUNT        16
/* Here we arbitrarily (there are only 16 switch positions, dammit) constrain
 * the length of a switch sequence to 16. */
#define MAX_SWITCH_SEQUENCE 16
typedef char SWITCH_SEQUENCE[MAX_SWITCH_SEQUENCE];


/* A phase and amplitude compensation array C takes four uncompensated channel
 * inputs X[i] and produces four compensated channel outputs Y[i] via a two
 * tap filter computing
 * 
 *      Y[i]_t = C[i][0] * X[i]_t  +  C[i][1] * X[i]_{t-1}  .
 *
 * Values in the array are signed 18 bit values scaled with 1.0 = 0x10000, so
 * the dynamic range is [-2..2). */
typedef int PHASE_ENTRY[2];
typedef PHASE_ENTRY PHASE_ARRAY[BUTTON_COUNT];
#define PHASE_UNITY 0x10000

/* A demultiplexing array P takes four raw ADC channel inputs X[i] and
 * produces four demultiplexed (and possibly crosstalk compensated) button
 * outputs Z[j] via the matrix multiplication
 * 
 *      Z[j] = SUM_i P[j][i] * Y[i]  .
 *      
 * Values in the array are signed 18 bit values. */
typedef int DEMUX_ARRAY[BUTTON_COUNT][BUTTON_COUNT];


/* Returns true iff the "brilliance" option is installed.  Mostly this can be
 * hidden from non-hardware code, but in certain cases different actions need
 * to be taken.
 *     The "brilliance" option consists of a 16-bit ADC with much improved RF
 * handling and a different range of attenuator settings: attenuators in
 * Libera Brilliance cover a much narrower range of values. */
bool Brilliance();

/* Writes a new attenuation value.  The attenuation will not be updated until
 * CommitDscState() is called. */
bool WriteAttenuation(int NewAttenuation);

/* Writes a sequence of switches.  This is an array of switch numbers (each
 * switch number in the range 0-15).  The length of the sequence must be a
 * power of 2 between 1 and 16 (inclusive).
 *    The active switch sequence will not be updated until CommitDscState()
 * is called. */
bool WriteSwitchSequence(
    const SWITCH_SEQUENCE NewSwitches, unsigned int NewLength);

/* Writes the phase correction matrix for one switch position.  The array is
 * diagonal and acts on button values.
 *     The phase array matrices will not be updated until CommitDscState()
 * is called. */
void WritePhaseArray(int Switch, const PHASE_ARRAY Array);

/* Writes the demultiplexing and and crosstalk correction matrix for one
 * switch position.
 *     The correction matrices will not be updated until CommitDscState() is
 * called. */
void WriteDemuxArray(int Switch, const DEMUX_ARRAY Array);

/* This writes the state establised by the following routines into the FPGA
 * in a single atomic operation:
 *      WriteAttenuation        Attenuator settings
 *      WriteSwitchSequence     Switching sequence
 *      WritePhaseArray         Phase and amplitude correction matrices
 *      WriteDemuxArray         Demultiplexing and crosstalk correction */
bool CommitDscState();


/* This selects between internal and external triggering of the rotating
 * switches.  If external triggering is selected the "machine clock" input is
 * used to generate the switch cycle. */
bool WriteSwitchTriggerSelect(bool ExternalTrigger);

/* Programmes the delay from the external machine clock to the switch
 * revolution.  This can be useful for moving the switch position relative to
 * the fill pattern which can affect noise. */
bool WriteSwitchTriggerDelay(int Delay);




/*****************************************************************************/
/*                                                                           */
/*                          Generic Helper Macros                            */
/*                                                                           */
/*****************************************************************************/

#define TEST_OK(test, message...) \
    ( { \
        bool __ok__ = (test); \
        if (!__ok__)  printf(message); \
        __ok__; \
    } )

/* Helper macro for OS I/O commands which return -1 on error and set errno.
 * Arguments are:
 *      var     Variable to receive result of command
 *      error   Error message to print on error
 *      command I/O command to call
 *      args    Arguments to I/O command
 * Return true on success, prints an error message and returns false if the
 * I/O command fails. */
#define TEST_IO(var, error, command, args...) \
    ( { \
        var = (command)(args); \
        if ((int) var == -1)  perror(error); \
        (int) var != -1; \
    } )

/* Much the same as for TEST_IO, but for I/O commands which return 0 on
 * success and an ignorable non-zero code on failure.  No variable needs to be
 * specified. */
#define TEST_RC(error, command, args...) \
    ( { \
        bool __ok__ = (command)(args) == 0; \
        if (!__ok__)  perror(error); \
        __ok__; \
    } )

/* An even more simplified version of TEST_RC, where the error string is
 * simply the function name. */
#define TEST_(command, args...)  TEST_RC(#command, command, args)


/* A rather randomly placed helper routine.  This and its equivalents are
 * defined all over the place, but there doesn't appear to be a definitive
 * definition anywhere. */
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof((a)[0]))
