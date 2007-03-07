# This file is part of the Libera EPICS Driver,
# Copyright (C) 2005-2007 Michael Abbott, Diamond Light Source Ltd.
#
# The Libera EPICS Driver is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or (at your
# option) any later version.
#
# The Libera EPICS Driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
#
# Contact:
#      Dr. Michael Abbott,
#      Diamond Light Source Ltd,
#      Diamond House,
#      Chilton,
#      Didcot,
#      Oxfordshire,
#      OX11 0DE
#      michael.abbott@diamond.ac.uk

# Python script for building Libera database

import sys
from math import *

# It is important to import support before importing epics, as the support
# module initialises epics (and determines which symbols it exports!)
from support import * 
from epics import *



# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------


# Boilerplate record generation.  Here is where the various types of records
# that can be generated are defined.

MAX_INT = 2**31 - 1
MAX_mm  = 10
MAX_nm  = MAX_mm * 10**6         # 10^7 nm = 10 mm
MAX_S   = MAX_INT
KB      = 1024
MB      = KB*KB

def RAW_ADC(length):
    return [
        Waveform('RAW' + channel, length,
            DESC = 'Raw ADC for channel %s' % channel,
            LOPR = -2048, HOPR = 2047)
        for channel in '1234']

def IQ_wf(length):
    return [
        Waveform('WF' + button + axis, length,
            DESC = '%s quadrature %s for button %s' % (
                ChannelName(), axis, button),
            LOPR = -MAX_S, HOPR = MAX_S)
        for button in 'ABCD' for axis in 'IQ']

def ABCD_wf(length):
    return [
        Waveform('WF' + button, length,
            DESC = '%s amplitude for button %s' % (ChannelName(), button),
            LOPR = 0, HOPR = MAX_S)
        for button in 'ABCD']

def ABCD_():
    return [
        longIn(button, 0, MAX_S,
            DESC = '%s button %s intensity' % (ChannelName(), button))
        for button in 'ABCD']

def XYQS_wf(length, prefix='WF'):
    return [
        Waveform(prefix + position, length,
            DESC = '%s %s position' % (ChannelName(), position),
            LOPR = -MAX_nm, HOPR = MAX_nm, EGU = 'nm')
        for position in 'XYQ'] + [
        Waveform(prefix + 'S', length,
            DESC = '%s total button intensity' % ChannelName(),
            LOPR = 0, HOPR = MAX_S)]

def XYQS_(prec, logMax=0, suffix=''):
    sl = [
        Libera.longin('S' + suffix, MDEL = -1,
            DESC = '%s total button intensity' % ChannelName())]
    if logMax:
        sl.append(records.calc('SL' + suffix,
            DESC = '%s nominal power' % ChannelName(),
            CALC = '20*(LOG(A)-%g)' % logMax,
            INPA = sl[0],
            MDEL = -1,
            LOPR = -50,   HOPR = 0,
            EGU  = 'dB',  PREC = 0))
    return [
        aIn(position + suffix, -MAX_mm, MAX_mm, 1e-6, 'mm', prec,
            DESC = '%s %s position' % (ChannelName(), position))
        for position in 'XYQ'] + sl
        
        

def Enable():
    boolOut('ENABLE', 'Disabled', 'Enabled',
        DESC = 'Enable %s mode' % ChannelName())

def Trigger(MC, positions, TRIG='TRIG', DONE='DONE'):
    # If MC is requested then generate MC machine clock records as well.
    # These return the 64 bit revolution clock as a pair of 32 bit values.
    if MC:
        positions = positions + [
            longIn('MCL', EGU='turns', DESC = 'Revolution clock (low)'),
            longIn('MCH', EGU='turns', DESC = 'Revolution clock (high)')]

    
    # The DONE record must be processed after all other triggered records are
    # processed: this is used as an interlock to synchronise with the Libera
    # driver.
    done = Libera.bo(DONE, DESC = 'Report trigger done')
    trigger = Libera.bi(TRIG, DESC = 'Trigger processing',
        SCAN = 'I/O Intr',
        TSE  = -2,          # Ensures that device timestamp is used
        FLNK = create_fanout(TRIG + 'FAN', *positions + [done]))
    for record in positions:
        record.TSEL = trigger.TIME

        
# ----------------------------------------------------------------------------
#           Libera Data Capture Mode Definitions
# ----------------------------------------------------------------------------
        

# First turn snapshot records.  Access to position data immediately following
# the trigger for use on transfer paths and during injection.
def FirstTurn():
    LONG_LENGTH = 1024
    SHORT_LENGTH = LONG_LENGTH / 4

    SetChannelName('FT')
    Enable()
    
    maxadc = longIn('MAXADC', 0, 2048,
        DESC = 'Maximum ADC reading',
        HSV  = 'MINOR',  HIGH = 1450,   # -3dB full scale
        HHSV = 'MAJOR',  HIHI = 1700)   # -1.6dB full scale

    charge = aIn('CHARGE', 0, 2000, 1e-6, 'nC', 2,
        DESC = 'Charge of bunch train')

    Trigger(False, 
        # Raw waveforms as read from the ADC rate buffer
        RAW_ADC(LONG_LENGTH) + [maxadc, charge] +
        # ADC data reduced by 1/4 by recombination
        ABCD_wf(SHORT_LENGTH) +
        # Synthesised button positions from windowed averages
        ABCD_() + 
        # Final computed positions with logarithmic scale.
        XYQS_(1, logMax = log10(2**30 * 0.582217 * sqrt(2))))

    # Sample window control
    longOut('OFF', 0, SHORT_LENGTH - 1,
        DESC = 'Sample window start position')
    longOut('LEN', 1, SHORT_LENGTH,
        DESC = 'Sample window length')

    UnsetChannelName()


# Booster ramp support records.  Decimated waveforms for overview of entire
# 100ms booster ramp.
def Booster():
    SHORT_LENGTH = Parameter('BN_SHORT')    # 190  = BN_LONG / 16
    LONG_LENGTH  = Parameter('BN_LONG')     # 3040 = BN_SHORT * 16

    SetChannelName('BN')
    Enable()
    
    Trigger(True, 
        # IQ data
        IQ_wf(LONG_LENGTH) + 
        # Raw decimated /64 button values
        ABCD_wf(LONG_LENGTH) +
        # Decimated /64 positions
        XYQS_wf(LONG_LENGTH) +
        # Decimated /1024 positions
        XYQS_wf(SHORT_LENGTH, 'WFS'))

    # Axes for user friendly graphs, used to label the WF and WFS waveforms.
    # Each labels the time axis in milliseconds.
    Waveform('AXIS',  LONG_LENGTH,  'FLOAT', PINI='YES',
        DESC = 'BN long waveform axis')
    Waveform('AXISS', SHORT_LENGTH, 'FLOAT', PINI='YES',
        DESC = 'BN short waveform axis')

    UnsetChannelName()


# Free running short (typically 2048) turn-by-turn buffer.  
def FreeRunning():
    LENGTH = Parameter('FR_LENGTH')
    
    SetChannelName('FR')
    Enable()

    # In this mode we provide all the available data: raw IQ, buttons and
    # computed positions.
    Trigger(True, IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH))
    
    UnsetChannelName()
        

# Postmortem fixed length buffer.  This mode is always enabled.
def Postmortem():
    LENGTH = 16384
    
    SetChannelName('PM')

    # All turn-by-turn data is provided.
    Trigger(True, IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH))
    
    UnsetChannelName()
        

# Turn-by-turn snapshot records.  Access to long waveforms captured on
# request.  Typically used for tune measurements.  Up to 200,000 points can
# be captured in one request.
def TurnByTurn():
    LONG_LENGTH   = Parameter('TT_LONG')
    WINDOW_LENGTH = Parameter('TT_WINDOW')
    
    SetChannelName('TT')
    
    # Number of points successfully captured by the last trigger.
    captured = longIn('CAPTURED', 0, LONG_LENGTH,
        DESC = 'TT points captured')
    # Write a one to request a refill of the buffer.
    # Note that historically this field is called ARM rather than ARM_S, and
    # so we have to use Libera.bo directly rather boolOut!
    arm = Libera.bo('ARM', ZNAM = 'Not armed', ONAM = 'Trigger enabled',
        DESC = 'Write one to enable capture')
    # This indicates whether the buffer has been filled.
    ready = boolIn('READY', 'No waveform', 'Waveform ready',
        DESC = 'Waveform captured',
        SCAN = 'I/O Intr', PINI = 'YES')
        
    # When the ready record makes a transition into the ready state re-enable
    # the arm record by setting it to zero.
    rearm = records.calcout('REARM',
        DESC = 'Clear ready on ARM',
        OUT  = arm,
        INPA = ready,
        CALC = '!A',
        OOPT = 'Transition To Zero',
        DOPT = 'Use CALC')
    ready.FLNK = create_fanout('FANA', rearm, captured)

    # Readout window and total capture length
    longOut('LENGTH', 1, WINDOW_LENGTH,
        DESC = 'TT readout window length')
    longOut('CAPLEN', 1, LONG_LENGTH,
        DESC = 'TT length of capture')
    # Readout window offset together with a readback to be used as a readout
    # interlock.
    longOut('OFFSET', 0, LONG_LENGTH,
        DESC = 'TT set readout window offset')
    offset = longIn('OFFSET', 0, LONG_LENGTH, PINI = 'YES',
        DESC = 'TT readout offset readback')

    Trigger(True, 
        # Raw I and Q values
        IQ_wf(WINDOW_LENGTH) + 
        # Button values
        ABCD_wf(WINDOW_LENGTH) +
        # Computed positions
        XYQS_wf(WINDOW_LENGTH) +
        [offset])

    UnsetChannelName()

    
# Slow acquisition: position updates at 10Hz.
def SlowAcquisition():
    SetChannelName('SA')
    power = aIn('POWER', -80, 10, 1e-6, 'dBm', 3,
        DESC = 'Absolute input power')
    current = aIn('CURRENT', 0, 500, 1e-5, 'mA', 3,
        DESC = 'SA input current')
    Trigger(False,
        ABCD_() + XYQS_(4) + XYQS_(4, suffix='C') + [power, current])
    UnsetChannelName()



# ----------------------------------------------------------------------------
#           Configuration
# ----------------------------------------------------------------------------
        

# Configuration control records.  Used for setting button or stripline
# geometry and 
def Config():
    SetChannelName('CF')

    # Control enabling of this BPM.
    global globalBpmEnable
    globalBpmEnable = boolOut('ENABLED', 'BPM Disabled', 'BPM Enabled',
        ZSV  = 'MAJOR',     OSV  = 'NO_ALARM',  PINI = 'YES',
        DESC = 'Enable Libera')

    # Geometry calibration control
    boolOut('DIAG', 'Vertical', 'Diagonal', DESC = 'Button orientation')
    aOut('KX', 0, 32,   EGU  = 'mm', DESC = 'X scaling')
    aOut('KY', 0, 32,   EGU  = 'mm', DESC = 'Y scaling')
    aOut('KQ', 0, 32,   EGU  = 'mm', DESC = 'Q scaling')

    # BPM origin control.  We support three separate displacements:
    #   1.  Beam Based Alignment offset.
    #   2.  Beam Current Dependency offset
    #   3.  "Golden Orbit" offset
    # Only 1 is stored as part of the persistent state.  1+2 define the
    # nominal origin (used for interlocks).
    aOut('BBA_X',    -16, 16, EGU = 'mm', DESC = 'Beam based X origin')
    aOut('BBA_Y',    -16, 16, EGU = 'mm', DESC = 'Beam based Y origin')
    aOut('BCD_X',    -16, 16, EGU = 'mm', DESC = 'Current dependent X origin')
    aOut('BCD_Y',    -16, 16, EGU = 'mm', DESC = 'Current dependent Y origin')
    aOut('GOLDEN_X', -16, 16, EGU = 'mm', DESC = 'Golden orbit X origin')
    aOut('GOLDEN_Y', -16, 16, EGU = 'mm', DESC = 'Golden orbit Y origin')
    
    # Channel gain settings.  Only applies to first turn mode.
    aOut('G0', 0, 1.5,  ESLO = 2**-30, DESC = 'Channel 0 gain adjustment')
    aOut('G1', 0, 1.5,  ESLO = 2**-30, DESC = 'Channel 1 gain adjustment')
    aOut('G2', 0, 1.5,  ESLO = 2**-30, DESC = 'Channel 2 gain adjustment')
    aOut('G3', 0, 1.5,  ESLO = 2**-30, DESC = 'Channel 3 gain adjustment')

    # Configure automatic switch state
    autoswitch = boolInOut('AUTOSW', 'Manual', 'Automatic',
        DESC = 'Configure rotating switches')
    # Select switch to use when automatic switching off
    longOut('SETSW', 0, 15, DESC = 'Fixed multiplexor switch')
    # Switch trigger selection and delay from trigger
    boolOut('TRIGSW', 'Internal', 'External',
        DESC = 'Switching trigger source')
    longOut('DELAYSW', 0, DESC = 'Switches trigger delay')
    
    # DSC control
    mbbInOut('DSC',
        ('Fixed gains', 0),     # Use last good DSC settings
        ('Unity gains', 1),     # Disable DSC, use fixed gains
        ('Automatic', 2),       # Run DSC
        DESC = 'Digitial Signal Conditioning')
    boolOut('WRITEDSC', 'Save DSC', DESC = 'Write DSC state file')
    
    # Control attenuation
    longOut('ATTEN', 0, 62, EGU = 'dB', DESC = 'Attenuator setting')

    # Scaling factor for conversion to bunch charge and stored current.
    aOut('ISCALE', 0, 20000, 
        DESC = 'Input current at 0dBm power',
        EGU  = 'mA', ESLO = 1e-5, PREC = 1)

    UnsetChannelName()


# Interlock control records.  Used for configuring interlock operation.
def Interlock():
    SetChannelName('IL')

    # Interlock window limits in mm
    aOut('MINX', -5, 5, EGU = 'mm', DESC = 'Interlock window min X')
    aOut('MAXX', -5, 5, EGU = 'mm', DESC = 'Interlock window max X')
    aOut('MINY', -5, 5, EGU = 'mm', DESC = 'Interlock window min Y')
    aOut('MAXY', -5, 5, EGU = 'mm', DESC = 'Interlock window max Y')

    # Interlock control state.  This tracks the internal state, but can also
    # be reset externally.
    boolInOut('ENABLE', 'Disabled', 'Enabled',
        DESC = 'Interlock master enable')

    # Interlock current threshold: interlock will automatically switch on
    # when this threshold is exceeded.
    aOut('ION', 0, 20000,
        DESC = 'Interlock on threshold',
        EGU  = 'mA', ESLO = 1e-5, PREC = 1)
    aOut('IOFF', 0, 20000,
        DESC = 'Interlock off threshold',
        EGU  = 'mA', ESLO = 1e-5, PREC = 1)

    # ADC overflow detection is also supported: this runs independently of
    # current limit triggered interlocking.  This mode is independently
    # enabled and configured.
    boolOut('OVERFLOW', 'Disabled', 'Enabled',
        DESC = 'Enable ADC overflow detect')
    longOut('OVER', 0, 2048,
        DESC = 'ADC overflow threshold')
    longOut('TIME', 1, 1024,
        DESC = 'ADC overflow duration')

    # Interlock holdoff delay
    longOut('HOLDOFF',  0, 1000, DESC = 'Interlock holdoff delay')
    longOut('IHOLDOFF', 0, 1000, DESC = 'Current holdoff delay')
    
    # Interlock state.  This is a bit nasty: we get repeated triggers on TRIG
    # while the interlock is active (ie, reporting signal bad).  The records
    # POKE_STATE simply acts to relay the trigger state to STATE, which
    # automatically resets itself after half a second if no triggers are
    # received.
#    mbbIn('REASON')

    trigger = boolIn('TRIG', '', 'Trigger',
        DESC = 'Interlock dropped event',
        SCAN = 'I/O Intr') 
    state = records.bo('STATE',
        HIGH = 0.5,             # Reset to low after 0.5 seconds
        VAL  = 0,  PINI = 'YES',
        OMSL = 'supervisory',
        ZNAM = 'Ready',         ZSV  = 'NO_ALARM',
        ONAM = 'Interlocked',   OSV  = 'MAJOR')
    trigger.FLNK = create_dfanout('POKE', PP(state), 
        DOL  = trigger, OMSL = 'closed_loop')
    
    UnsetChannelName()


    
# -----------------------------------------------------------------------------



def AggregateSeverity(name, description, *recs):
    ''' Aggregates the severity of all the given records into a single record.
    The value of the record is constant, but its SEVR value reflects the
    maximum severity of all of the given records.'''
    
    assert len(recs) <= 12, 'Too many records to aggregate'
    return records.calc(name,
        CALC = 1, DESC = description,
        # Assign each record of interest to a calc record input with MS.
        # This then automatically propagates to the severity of the whole
        # record.
        **dict(zip(['INP'+c for c in 'ABCDEFGHIJKL'], map(MS, recs))))


    
# Clock configuation control records.  Used for managing clocks and
# synchronisation.
def Clock():
    SetChannelName('CK')

    # Configure whether to use system or NTP time
    boolOut('TIMESTAMP', 'NTP time', 'System time',
        DESC = 'Configure timestamp source')
    # Control LMTD detune, intermediate frequency and phase
    longOut('DETUNE', -1000, 1000, DESC = 'Sample clock detune')
    longOut('IFOFF',  -1000, 1000, DESC = 'IF clock detune')
    longOut('PHASE', DESC = 'Phase offset')
    
    # Commands for clock synchronisation
    boolOut('MC_SYNC', 'Synchronise', None, DESC = 'Synchronise machine clock')
    boolOut('SC_SYNC', 'Synchronise', None, DESC = 'Synchronise system clock')


    # Create the tick health monitor.  This records the number of seconds
    # since the last recorded trigger and signals an alarm according to how
    # long the delay has been.
    tick = records.calc('TICK',
        SCAN = '.1 second',
        CALC = 'A+0.1',
        EGU  = 's', PREC = 1,
        HIGH = 1,   HSV  = 'MINOR')
    tick.INPA = tick
    tick_reset = records.calcout('TICK_CALC',
        CALC = '0',
        OUT  = tick,
        OOPT = 'Every Time',
        DOPT = 'Use CALC')

    
    # Group together all the records that affect the alarm status. */
    mc_health = [
        # MC monitoring PVS
        mbbIn('MC_LOCK',
            ('No Clock',        0, 'MAJOR'),
            ('Seek Frequency',  1, 'MAJOR'),
            ('Slewing',         2, 'MINOR'),
            ('Phase Locked',    3, 'NO_ALARM'),
            DESC = 'Machine clock lock status'),
        mbbIn('MC_SYNC',
            ('Not Synched',     0, 'MINOR'),
            ('Waiting Trigger', 1, 'MINOR'),
            ('Synchronised',    2, 'NO_ALARM'),
            DESC = 'Machine clock sync state')]

    sc_health = [
        # LSTD monitoring PVS
        boolIn('SC_LOCK', 'Unlocked', 'Locked',
            DESC = 'System clock lock status',
            ZSV  = 'MINOR',     OSV  = 'NO_ALARM'),
        mbbIn('SC_SYNC',
            ('Not Synched',     0, 'MINOR'),
            ('Waiting Trigger', 1, 'MINOR'),
            ('Synchronised',    2, 'NO_ALARM'),
            DESC = 'System clock sync state')]
    # For the moment we don't aggregate SC health: it just doesn't stay
    # healthy for long enough!  The problem is (almost certainly) that the
    # lstd loses lock due to excessive oscillation, thus causing
    # synchronisation to be dropped.
    clock_health = AggregateSeverity('HEALTH', 'Clock status',
        *mc_health + [tick])    # + sc_health

    # This trigger receives the normal machine trigger.
    Trigger(True, [
        tick_reset,
        stringIn('TIME_NTP', DESC = 'NTP time'),
        stringIn('TIME_SC',  DESC = 'System clock time')],
        TRIG = 'TIME', DONE = 'TIME_DONE')
    
    Trigger(False, [
        longIn('DAC', 0, 65535, DESC = 'LMTD VCXO DAC setting'),
        longIn('PHASE_E', DESC = 'LMTD phase error'),
        longIn('FREQ_E', DESC = 'LMTD frequency error')] + 
        mc_health + sc_health + [clock_health,])
    
    UnsetChannelName()

    
def Voltages():
    '''Prepares records for all the eight voltage readouts.  Two values are
    returned: all records generated, in the correct order for scan processing,
    and the aggregated health record, to be combined with the global health
    status record.'''

    def VoltageSensor(i, description, nominal, limits=True):
        LoLo = nominal - 0.10 * abs(nominal)
        Low  = nominal - 0.05 * abs(nominal)
        High = nominal + 0.05 * abs(nominal)
        HiHi = nominal + 0.10 * abs(nominal)
        if limits:
            LimitFields = dict(
                LOLO = LoLo,    LLSV = 'MAJOR',
                LOW  = Low,     LSV  = 'MINOR',
                HIGH = High,    HSV  = 'MINOR',
                HIHI = HiHi,    HHSV = 'MAJOR')
        else:
            LimitFields = dict()
        return aIn('VOLT%d' % i, Low, High,
            DESC = description,
            ESLO = 1e-3, EGU  = 'V', PREC = 3, **LimitFields)

    voltages = [
        VoltageSensor(1, 'Virtex core power supply',    1.5),
        VoltageSensor(2, '(Unused voltage)',            1.8, False),
        VoltageSensor(3, 'Virtex ADC interface',        2.5),
        VoltageSensor(4, 'SBC & SDRAM power supply',    3.3),
        VoltageSensor(5, 'PMC +5V power supply',        5.0),
        VoltageSensor(6, 'Fans power supply',           12.0),
        VoltageSensor(7, 'PMC -12V power supply',       -12.0),
        VoltageSensor(8, 'Attenuators & switches ctrl', -5.0)]

    
    health = AggregateSeverity('VOLTSOK', 'Voltage health', *voltages)
    health_display = records.mbbi('VHEALTH',
        DESC = 'Voltage health display',
        INP  = MS(health.SEVR),
        ZRVL = 0,   ZRST = 'Ok',
        ONVL = 1,   ONST = 'Warning',
        TWVL = 2,   TWST = 'Fault')

    return voltages + [health, health_display], health
    


def Sensors():
    SetChannelName('SE')

    temp = longIn('TEMP', 20, 60, 'deg C',
        DESC = 'Internal box temperature',
        HIGH = 45,      HSV  = 'MINOR',
        HIHI = 50,      HHSV = 'MAJOR',
        LOLO = 0,       LLSV = 'MINOR')
    fans = [longIn('FAN%d' % i, 0, 6000, 'RPM',
        DESC = 'Fan %d speed' % i,
        LOW  = 4000,    LSV  = 'MINOR',
        LOLO = 1000,    LLSV = 'MAJOR') for i in (1, 2)]
    memfree = aIn('FREE', 0, 64, 1./MB, 'MB', 2,
        DESC = 'Free memory',
        LOW  = 24,      LSV  = 'MINOR',
        LOLO = 16,      LLSV = 'MAJOR')
    ramfs = aIn('RAMFS', 0, 64, 1./MB, 'MB', 3,
        DESC = 'Temporary file usage',
        HIGH = 1,       HSV  = 'MINOR',
        HIHI = 16,      HHSV = 'MAJOR')
    cpu = aIn('CPU', 0, 100, 1e-3, '%', 1,
        DESC = 'CPU usage',
        HIGH = 80,      HSV  = 'MINOR',
        HIHI = 95,      HHSV = 'MAJOR')

    voltages, voltage_health = Voltages()
    
    uptime = aIn('UPTIME', 0, 24*3600*5, 1./3600, 'h', 2,
        DESC = 'Total system up time')
    epicsup = aIn('EPICSUP', 0, 24*3600*5, 1./3600, 'h', 2,
        DESC = 'Time since EPICS started')

    # Aggregate all the alarm generating records into a single "health"
    # record.  Only the alarm status of this record is meaningful.
    alarmsensors = fans + [temp, memfree, ramfs, cpu] #, voltage_health]
    health = AggregateSeverity('HEALTH', 'Aggregated health',
        *alarmsensors + [CP(globalBpmEnable)])
    
    allsensors = alarmsensors + voltages + [uptime, epicsup, health]
    
    boolOut('PROCESS', '', '',
        DESC = 'Sensors scan',
        SCAN = '10 second',
        PINI = 'YES',
        FLNK = create_fanout('FANOUT', *allsensors))
            
    UnsetChannelName()

    # Mirror the health record for backwards compatibility.
    records.bi('HEALTH', INP = MS(CP(health)))

    
def Miscellaneous():
    # The VERSION and BUILD strings are not tied to any "channel".
    Libera.stringin('VERSION', PINI = 'YES',
        DESC = 'Libera EPICS driver version')
    Libera.stringin('BUILD',   PINI = 'YES',
        DESC = 'EPICS driver build date')

    boolOut('REBOOT',  'Reboot',  None, DESC = 'Reboot Libera IOC')
    boolOut('RESTART', 'Restart', None, DESC = 'Restart EPICS driver')

    boolOut('CORE', 'Core dump', DESC = 'Force core dump!')

    

# Finally generate and output the supported records.
    
FirstTurn()         # FT - one position from somewhere within a 100us window
Booster()           # BN - 1024:1 (and 64:1) decimated waveforms
FreeRunning()       # FR - turn by turn, updating on every trigger
TurnByTurn()        # TT - very long waveforms, triggered once on demand
SlowAcquisition()   # SA - 10Hz low noise position
Postmortem()        # PM - fixed length pre-postmortem trigger waveforms

Config()            # CF - general configuration records
Interlock()         # IL - interlock configuration records
Clock()             # CK - clock monitoring and control
Sensors()           # SE - temperatures, fan speeds, memory and CPU usage etc
Miscellaneous()     # Other records

WriteRecords(sys.argv[1])
