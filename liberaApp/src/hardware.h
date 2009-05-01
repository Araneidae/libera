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


#include "driver/libera.h"
/* Unfortunately Instrumentation Technologies have been rather sloppy in the
 * 1.80 to 2.00 upgrade, and managed to badly break backwards compatibility.
 * Fortunately this was done on a architecture boundary, so we can detect
 * this on the presence of the __ARM_EABI__ flag.  Unfortunately they also
 * screwed up on the 2.00 to 2.02 upgrade, inserting LIBERA_CFG_PMDEC into
 * the middle of a block of definitions, so we'll need to take this into
 * account as well! */
#ifdef __ARM_EABI__
#define __EBPP_H_2
#include "driver/ebpp.h"
#else
#define __EBPP_H_1
#include "driver/ebpp-1.80.h"
#endif


/* Exports for libera device driver interface routines. */



/*****************************************************************************/
/*                                                                           */
/*                             Data Structures                               */
/*                                                                           */
/*****************************************************************************/


/* We'll try to distinguish iterations over channels from iterations over
 * buttons. */
#define BUTTON_COUNT    4
#define CHANNEL_COUNT   4


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
typedef short ADC_ROW[CHANNEL_COUNT];
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
bool WriteRawRegister(
    unsigned int Address, unsigned int Value, unsigned int Mask = 0xFFFFFFFF);
bool ReadRawRegister(unsigned int Address, unsigned int &Value);
#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                   Reading waveform data from the FPGA.                    */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Waveforms are timestamped with both the machine and the system time of the
 * start of the waveform. */
typedef libera_timestamp_t LIBERA_TIMESTAMP;


/* Reads a waveform of the given length (number of rows) using the given
 * decimation into the given block of data.  Returns the number of rows
 * actually read.
 *     At present only decimation values of 1 or 64 are suppported. */
size_t ReadWaveform(
    int Decimation, size_t WaveformLength, LIBERA_ROW * Data,
    LIBERA_TIMESTAMP & Timestamp, int Offset = 0);

/* Reads the postmortem buffer. */
size_t ReadPostmortem(
    size_t WaveformLength, LIBERA_ROW * Data, LIBERA_TIMESTAMP & Timestamp);

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


/* The interlock hardware can operate in one of the following three modes. */
enum LIBERA_ILKMODE
{
    LIBERA_ILK_DISABLE = 0,         // Interlock disabled
    LIBERA_ILK_ENABLE = 1,          // Interlock unconditionally enabled
    LIBERA_ILK_ENABLE_GAINDEP = 3,  // Position ilk off, ADC overflow ilk on
};


/* Interlock settings.   As the CSPI interface requires that we set all the
 * parameters together, we present the interface in one piece. */
bool WriteInterlockParameters(
    LIBERA_ILKMODE mode,
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
bool SetSystemClockTime(const struct timespec & NewTime);
bool GetClockState(bool &LmtdLocked, bool &LstdLocked);


/* Programs the set of Libera events which will be notified. */
bool SetEventMask(int EventMask);
/* Returns the next event from the Libera event queue.  Blocks until the next
 * event is available. */
bool ReadEvent(int &EventId, int &Parameter);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                      DSC Device Interface Routines.                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* There are sixteen possible switch positions. */
#define SWITCH_COUNT        16
/* Here we arbitrarily (there are only 16 switch positions, dammit) constrain
 * the length of a switch sequence to 16. */
#define MAX_SWITCH_SEQUENCE 16
typedef uint8_t SWITCH_SEQUENCE[MAX_SWITCH_SEQUENCE];


/* A phase and amplitude compensation array C takes four uncompensated channel
 * inputs X[i] and produces four compensated channel outputs Y[i] via a two
 * tap filter computing
 * 
 *      Y[i]_t = C[i][0] * X[i]_t  +  C[i][1] * X[i]_{t-1}  .
 *
 * Values in the array are signed 18 bit values scaled with 1.0 = 0x10000, so
 * the dynamic range is [-2..2). */
typedef int PHASE_ENTRY[2];
typedef PHASE_ENTRY PHASE_ARRAY[CHANNEL_COUNT];
#define PHASE_UNITY 0x10000

/* A demultiplexing array P takes four raw ADC channel inputs X[i] and
 * produces four demultiplexed (and possibly crosstalk compensated) button
 * outputs Z[j] via the matrix multiplication
 * 
 *      Z[j] = SUM_i P[j][i] * Y[i]  .
 *      
 * Values in the array are signed 18 bit values. */
typedef int DEMUX_ARRAY[BUTTON_COUNT][CHANNEL_COUNT];


/* Returns the maximum attenuation value: this is architecture dependent, and
 * can be 62 or 31 depending on whether Libera Electron or Brilliance is
 * present. */
int MaximumAttenuation();
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

/* Sets an internal delay from external triggers in sample clocks. */
bool WriteExternalTriggerDelay(int Delay);


/* Writes a filter coefficient for an IIR on the position interlock.  This
 * can be a value between 0 and 6 (inclusive), corresponding to a coefficient
 * of 2^-K for a one pole IIR (pole at z=1-2^-K). */
bool WriteInterlockIIR_K(int K);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        2.0 FPGA Specific Features                         */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Average of total intensity betwen two triggers and the number of samples
 * (in FA samples) between the two triggers. */
void GetTriggeredAverageSum(int &Sum, int &Samples);

/* Control of Moving Average Filter (if configured). */
bool WriteMafSettings(int Offset, int Delay);

/* Spike removal control parameters. */
bool WriteSpikeRemovalSettings(
    bool Enable, int AverageWindow, int AverageStop,
    int SpikeStart, int SpikeWindow);

/* Read spike removal debug buffer. */
#define SPIKE_DEBUG_BUFLEN  128
bool ReadSpikeRemovalBuffer(int Buffer[SPIKE_DEBUG_BUFLEN]);

/* Postmortem triggering control. */
enum PM_TRIGGER_MODE { PM_EXTERNAL, PM_INTERLOCK, PM_SETTINGS };
bool WritePostmortemTriggering(
    PM_TRIGGER_MODE Mode, int Xlow, int Xhigh, int Ylow, int Yhigh,
    int OverflowLimit, int OverflowDuration);



/* Pick up the generic helper test macros. */
#include "test_error.h"
