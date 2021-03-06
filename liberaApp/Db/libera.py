# This file is part of the Libera EPICS Driver,
# Copyright (C) 2005-2011 Michael Abbott, Diamond Light Source Ltd.
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
from __future__ import division

import sys
from math import *

from common import *


# ----------------------------------------------------------------------------
#           Libera Data Capture Mode Definitions
# ----------------------------------------------------------------------------

def dB(db):
    return 10.**(db/20.)


def MaxAdc(Name = 'MAXADC'):
    maxadc = longIn(Name, 0, MAX_ADC,
        DESC = 'Maximum ADC reading',
        HSV  = 'MINOR',  HIGH = int(MAX_ADC / dB(3)),    # 70.8 %
        HHSV = 'MAJOR',  HIHI = int(MAX_ADC / dB(1.6)))  # 83.2 %
    maxadc_pc = records.calc('%s_PC' % Name,
        DESC = 'Maximum ADC reading (%)',
        CALC = 'A/B',
        INPA = MS(maxadc),
        INPB = MAX_ADC / 100.,
        LOPR = 0,   HOPR = 100,
        PREC = 1,   EGU  = '%')
    return [maxadc, maxadc_pc]


# First turn snapshot records.  Access to position data immediately following
# the trigger for use on transfer paths and during injection.
def FirstTurn():
    LONG_LENGTH = 1024
    SHORT_LENGTH = LONG_LENGTH // 4 - 1

    SetChannelName('FT')
    Enable()

    maxadc, maxadc_pc = MaxAdc()
    charge = aIn('CHARGE', 0, 2000, 1e-6, 'nC', 2,
        DESC = 'Charge of bunch train')
    max_S = longIn('MAXS', DESC = 'Maximum S in waveform')

    Trigger(True,
        # Raw waveforms as read from the ADC rate buffer
        RAW_ADC(LONG_LENGTH) +
        [maxadc, maxadc_pc, charge, max_S] +
        # ADC data reduced by 1/4 by recombination
        ABCD_wf(SHORT_LENGTH) + XYQS_wf(SHORT_LENGTH) +
        # Buttons and positions computed within selected window
        ABCD_() + XYQS_(2))

    # Sample window control
    longOut('OFF', 0, SHORT_LENGTH - 1,
        DESC = 'Sample window start position')
    longOut('LEN', 1, SHORT_LENGTH,
        DESC = 'Sample window length')

    Waveform('AXIS', SHORT_LENGTH, 'FLOAT', PINI='YES',
        DESC = 'FT waveform axis')

    UnsetChannelName()


# Booster ramp support records.  Decimated waveforms for overview of entire
# 100ms booster ramp.
def Booster():
    SHORT_LENGTH = Parameter('BN_SHORT', 'Length of short BN waveform')
    LONG_LENGTH  = Parameter('BN_LONG', 'Length of long BN waveform')

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


# Statistics for waveforms
def WaveformStats(axis):
    return [
        aIn('MEAN%s' % axis, -1e4, 1e4, 1e-3,
            PREC = 2, EGU = 'um',
            DESC = 'Mean %s FR position' % axis),
        aIn('STD%s' % axis, 0, 1e4, 1e-3,
            PREC = 2, EGU = 'um',
            DESC = 'Standard dev. %s FR position' % axis),
        aIn('MIN%s' % axis, -1e4, 1e4, 1e-3,
            PREC = 2, EGU = 'um',
            DESC = 'Minimum %s FR position' % axis),
        aIn('MAX%s' % axis, -1e4, 1e4, 1e-3,
            PREC = 2, EGU = 'um',
            DESC = 'Maximum %s FR position' % axis),
        aIn('PP%s' % axis, 0, 2e4, 1e-3,
            PREC = 2, EGU = 'um',
            DESC = 'Peak to peak %s FR' % axis)]

# Computed tune statistics
def TuneStats(axis):
    tune_scale = 2**-11
    tune_stats = [
        aIn('TUNEI%s' % axis, 0, 1e6, tune_scale,
            EGU  = 'nm', PREC = 2,
            DESC = 'I component of %s tune' % axis),
        aIn('TUNEQ%s' % axis, 0, 1e6, tune_scale,
            EGU  = 'nm', PREC = 2,
            DESC = 'Q component of %s tune' % axis),
        aIn('TUNEMAG%s' % axis, 0, 1e6, tune_scale,
            EGU  = 'nm', PREC = 2,
            DESC = 'Magnitude of %s tune' % axis),
        aIn('TUNEPH%s' % axis, -180, +180, 360 * 2**-32,
            EGU  = 'deg', PREC = 1,
            DESC = 'Phase of %s tune' % axis)]

    aOut('TUNE%s' % axis, 0, 0.5, 2**-32,
        PREC = 5,
        FLNK = create_fanout('TUNEFAN%s' % axis, *tune_stats),
        DESC = 'Tune frequency to detect on %s' % axis)
    return tune_stats

def StatsXY():
    return \
        WaveformStats('X') + WaveformStats('Y') + \
        TuneStats('X') + TuneStats('Y')


# Free running short (typically 2048) turn-by-turn buffer.
def FreeRunning():
    LENGTH = Parameter('FR_LENGTH', 'Length of FR waveform')

    SetChannelName('FR')
    Enable()

    # In this mode we provide all the available data: raw IQ, buttons,
    # computed positions and statistics.
    Trigger(True,
        IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH) + StatsXY())

    # Trigger capture offset
    longOut('DELAY', DESC = 'Trigger capture offset')

    # Average length
    longOut('AVERAGE', 0, 16,
        DESC = 'Average samples as power of 2')
    boolOut('RESET', DESC = 'Restart average accumulation')
    boolOut('AUTOSTOP', 'Auto restart', 'Stop when done',
        DESC = 'Restart averaging when done?')
    boolOut('ALLUPDATE', 'Update when done', 'All updates',
        DESC = 'Update waveforms during averaging?')
    longIn('SAMPLES', SCAN = 'I/O Intr',
        DESC = 'Accumulated samples in average')

    UnsetChannelName()


# Turn-by-turn snapshot records.  Access to long waveforms captured on
# request.  Typically used for tune measurements.  Up to 200,000 points can
# be captured in one request.
def TurnByTurn():
    LONG_LENGTH   = Parameter('TT_LONG', 'Length of long TT capture')
    WINDOW_LENGTH = Parameter('TT_WINDOW', 'Length of TT readout waveform')

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
    records.longin('MAXLENGTH', VAL = WINDOW_LENGTH, PINI = 'YES',
        DESC = 'Max TT readout window length')
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
    # Trigger capture offset
    longOut('DELAY', DESC = 'Trigger capture offset')
    # Decimation control
    boolOut('DECIMATION', '1:1', '1:64',
        DESC = 'Decimation from turn-by-turn')
    # Controls whether to update windowed data on capture.
    boolOut('DOREFRESH', 'Stale Data', 'Update Data',
        DESC = 'Update displayed data on trigger')

    Trigger(True,
        # Raw I and Q values
        IQ_wf(WINDOW_LENGTH) +
        # Button values
        ABCD_wf(WINDOW_LENGTH) +
        # Computed positions
        XYQS_wf(WINDOW_LENGTH) +
        # Statistics
        StatsXY() +
        [offset])

    UnsetChannelName()


# Slow acquisition: position updates at 10Hz.
def SlowAcquisition():
    SetChannelName('SA')
    power = aIn('POWER', -80, 10, 1e-6, 'dBm', 3,
        DESC = 'Absolute input power')
    current = aIn('CURRENT', 0, 500, 1e-5, 'mA', 3,
        DESC = 'SA input current')
    Trigger(False, ABCD_() + ABCD_N() + XYQS_(4) + [power, current] + MaxAdc())
    UnsetChannelName()



# ----------------------------------------------------------------------------
#           Configuration
# ----------------------------------------------------------------------------


# Configuration control records.  Used for setting button or stripline
# geometry and a variety of other configuration settings.
def Config():
    # The number of attenuators depends on whether we're Electron or
    # Brilliance.
    ATTEN_COUNT = Parameter('ATTEN_COUNT', 'Number of attenuator settings')
    FIR_LENGTH  = Parameter('FIR_LENGTH', 'Length of FA decimation FIR')
    MAX_ATTEN = 62

    SetChannelName('CF')

    # Control enabling of this BPM.
    boolOut('ENABLED', 'BPM Disabled', 'BPM Enabled',
        ZSV  = 'MAJOR',     OSV  = 'NO_ALARM',  PINI = 'YES',
        DESC = 'Enable Libera')

    # Geometry calibration control
    boolOut('DIAG', 'Vertical', 'Diagonal', DESC = 'Button orientation')
    aOut('KX', 0, 32,   EGU  = 'mm', DESC = 'X scaling')
    aOut('KY', 0, 32,   EGU  = 'mm', DESC = 'Y scaling')

    # Scaling has no meaning for Q, but the zero point is worth defining
    aOut('Q_0', -1, 1,  ESLO = 1e-8, PREC = 4, DESC = 'Q offset')

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
    setsw = longOut('SETSW', 0, 15, DESC = 'Fixed multiplexor switch')
    setsw.FLNK = Waveform('PERM', 4,
        PINI = 'YES',
        DESC = 'Switch permutation')
    # Switch trigger selection and delay from trigger
    boolOut('TRIGSW', 'Internal', 'External',
        DESC = 'Switching trigger source')
    longOut('DELAYSW', 0, DESC = 'Switches trigger delay')

    # DSC control
    mbbInOut('DSC',
        ('Fixed gains', 0),     # Use last good DSC settings
        ('Unity gains', 1),     # Disable DSC, use fixed gains
        ('Automatic', 2),       # Run DSC
        DESC = 'Digital Signal Conditioning')

    # Control attenuation.  This is now a bit involved:
    #
    #   ATTEN:TRUE = ATTEN + ATTEN:DISP + ATTEN:OFFSET[ATTEN + ATTEN:DISP]
    #
    # Here ATTEN:DISP is used as a local offset to the attenuator setting --
    # this is designed to be used with a mix of Liberas, particularly useful
    # when dealing with a mix of Brilliance and Electron.
    #    The ATTEN:OFFSET array is used to correct for minor errors between
    # the selected attenuator value and its true reading.
    true_atten = aIn('ATTEN:TRUE', 0, MAX_ATTEN,
        EGU = 'dB',     PREC = 2,   PINI = 'YES',
        DESC = 'Corrected attenuator setting')
    longInOut('ATTEN', 0, MAX_ATTEN, EGU = 'dB',
        FLNK = true_atten,
        DESC = 'Attenuator setting')
    WaveformOut('ATTEN:OFFSET',
        length = ATTEN_COUNT,   FTVL = 'FLOAT', EGU = 'dB',
        FLNK = true_atten,
        DESC = 'Attenuator correction offset')
    longOut('ATTEN:DISP', -MAX_ATTEN, MAX_ATTEN, EGU = 'dB',
        FLNK = true_atten,
        DESC = 'Attenuator displacement')

    boolOut('ATTEN:AGC', 'AGC off', 'AGC on',
        DESC = 'Enables attenuator AGC')
    longOut('ATTEN:AGC:DN', 0, 100, '%',
        DESC = 'AGC drop threshold')
    longOut('ATTEN:AGC:UP', 0, 100, '%',
        DESC = 'AGC raise threshold')

    # Scaling factor for conversion to bunch charge and stored current.
    aOut('ISCALE', 0, 20000,
        DESC = 'Input current at 0dBm power',
        EGU  = 'mA', ESLO = 1e-5, PREC = 1)

    # Notch filter enable
    boolOut('NOTCHEN', 'Disabled', 'Enabled',
        DESC = 'Enable/disable notch filters')
    filters = [
        WaveformOut('NOTCH1', length = 5,
            DESC = 'First notch filter coefficients'),
        WaveformOut('NOTCH2', length = 5,
            DESC = 'Second notch filter coefficients'),
        WaveformOut('FIR', length = FIR_LENGTH,
            DESC = 'FA decimation FIR filter')]

    # Filter reset is a little contrived: after the filters have been internally
    # reset the next record process for each filter will cause the EPICS
    # waveforms to be reinitialised.
    boolOut('RESETFA', 'Reset Filter',
        DESC = 'Reset FA filters to defaults',
        FLNK = create_fanout('TRIGFAN', *filters))

    # Internal trigger skew
    longOut('TRIGDLY', 0, (1<<12)-1, DESC = 'Internal trigger delay')

    UnsetChannelName()


# Interlock settings, duplicated between IL and PM modes.
def InterlockSettings():
    # Interlock window limits in mm
    aOut('MINX', -5, 5, EGU = 'mm', DESC = 'Interlock window min X')
    aOut('MAXX', -5, 5, EGU = 'mm', DESC = 'Interlock window max X')
    aOut('MINY', -5, 5, EGU = 'mm', DESC = 'Interlock window min Y')
    aOut('MAXY', -5, 5, EGU = 'mm', DESC = 'Interlock window max Y')

    # ADC overflow detection
    longOut('TIME', 1, 4095,
        DESC = 'ADC overflow duration')
    overflow = longOut('OVER', 0, MAX_ADC,
        DESC = 'ADC overflow threshold')
    overflow_pc = records.ao('OVER_PC_S',
        DESC = 'ADC overflow threshold (%)',
        OMSL = 'supervisory', DTYP = 'Raw Soft Channel',
        OUT  = PP(overflow),
        EGUL = 0,   EGUF = 100,   EGU  = '%', PREC = 1,
        DRVL = 0,           DRVH = 100.*(MAX_ADC-1.)/MAX_ADC,
        LINR = 'LINEAR',    ESLO = 100./MAX_ADC)
    overflow.FLNK = records.ao('OVER_C',
        OMSL = 'closed_loop', DTYP = 'Raw Soft Channel',
        DOL  = overflow,    OUT  = overflow_pc,
        DRVL = 0,           DRVH = MAX_ADC - 1,
        LINR = 'LINEAR',    ESLO = MAX_ADC/100.,
        PINI = 'YES')


# Interlock control records.  Used for configuring interlock operation.
def Interlock():
    SetChannelName('IL')

    # Common shared interlock settings
    InterlockSettings()
    # True interlocks
    true_interlocks = [
        aIn('MINX', -5, 5, EGU = 'mm', PREC = 3, MDEL = 0, ADEL = 0,
            DESC = 'Current interlock min X'),
        aIn('MAXX', -5, 5, EGU = 'mm', PREC = 3, MDEL = 0, ADEL = 0,
            DESC = 'Current interlock max X'),
        aIn('MINY', -5, 5, EGU = 'mm', PREC = 3, MDEL = 0, ADEL = 0,
            DESC = 'Current interlock min Y'),
        aIn('MAXY', -5, 5, EGU = 'mm', PREC = 3, MDEL = 0, ADEL = 0,
            DESC = 'Current interlock max Y')]

    # Extra secondary interlock settings
    aOut('MINX2', -5, 5, EGU = 'mm', DESC = 'Secondary interlock min X')
    aOut('MAXX2', -5, 5, EGU = 'mm', DESC = 'Secondary interlock max X')
    aOut('MINY2', -5, 5, EGU = 'mm', DESC = 'Secondary interlock min Y')
    aOut('MAXY2', -5, 5, EGU = 'mm', DESC = 'Secondary interlock max Y')
    mbbIn('WINDOW', ('Primary', 0), ('Secondary', 1),
        SCAN = '.1 second',
        FLNK = create_fanout('FANILK', *true_interlocks),
        DESC = 'Currently active interlock')

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


    # Interlock holdoff delay
    longOut('HOLDOFF',  0, 1000, DESC = 'Interlock holdoff delay')
    longOut('IHOLDOFF', 0, 1000, DESC = 'Current holdoff delay')
    # IIR constants
    longOut('IIRK', 0, 6, DESC = 'Interlock ADC IIR constant')
    longOut('XY:IIRK', 0, 255, DESC = 'Interlock X/Y IIR constant')

    # Interlock state.  This is a bit nasty: we get repeated triggers on TRIG
    # while the interlock is active (ie, reporting signal bad).  The record
    # POKE simply acts to relay the trigger state to STATE, which
    # automatically resets itself after half a second if no triggers are
    # received.
    state = records.bo('STATE',
        HIGH = 0.5,             # Reset to low after 0.5 seconds
        VAL  = 0,  PINI = 'YES',
        OMSL = 'supervisory',
        ZNAM = 'Ready',         ZSV  = 'NO_ALARM',
        ONAM = 'Interlocked',   OSV  = 'MAJOR')
    poke = create_dfanout('POKE', PP(state), VAL = 1)

    # Figure out whether this is a fresh interlock event.  This needs to be
    # processed before the poke record above.
    raw_reason = longIn('RAW_REASON', DESC = 'Interlock reason')
    reason = records.mbbiDirect('REASON')
    reason_check = records.calcout('CHK_REASON',
        INPA = state,       CALC = 'A',     OOPT = 'When Zero',
        DOPT = 'Use OCAL',  OCAL = 'B',     INPB = raw_reason,
        OUT  = PP(reason))

    Trigger(False, [raw_reason, reason_check, poke])

    interlock_test = boolOut('TEST', 'Normal', 'Interlock Test',
        ZSV  = 'NO_ALARM',  OSV  = 'MAJOR',
        VAL  = 0,   PINI = 'YES',
        DESC = 'Drop interlock for testing')
    ExtraHealthRecords.append(CP(interlock_test))

    UnsetChannelName()


# Postmortem fixed length buffer.  This mode is always enabled.
def Postmortem():
    LENGTH = 16384

    def Overflow(name):
        return [
            longIn('%s_OFFSET' % name, 0, LENGTH,
                DESC = '%s overflow offset' % name),
            boolIn('%s_OFL' % name, 'No overflow', 'Overflowed',
                DESC = '%s overflow occurred' % name)]

    SetChannelName('PM')

    # Retrigger control.
    ready = boolIn('READY', 'Quiescent', 'Trigger enabled', PINI = 'YES',
        DESC = 'Ready to capture postmortem')
    boolOut('MODE', 'All', 'One Shot', FLNK = ready,
        DESC = 'Postmortem trigger mode')
    boolOut('REARM', 'Arm', FLNK = ready,
        DESC = 'Process to arm one shot')

    # All turn-by-turn data is provided.  We also provide digests of the
    # postmortem reason.
    Trigger(True,
        IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH) +
        Overflow('X') + Overflow('Y') + Overflow('ADC') +
        [Waveform('FLAGS', LENGTH, 'UCHAR',
            DESC = 'Interlock overflow flags'), ready])

    # Special postmortem configuration control.  These correspond to
    # interlock PVs, but can be used for separate control of PM events (but
    # only if FPGA 2 support is loaded).
    mbbOut('SOURCE', ('External', 0), ('Interlock', 1), ('Settings', 2),
        DESC = 'Postmortem trigger source')
    longOut('OFFSET', DESC = 'PM trigger offset')
    InterlockSettings()

    UnsetChannelName()


# Compensation matrices are all up to 8x4x2 -- 8 (or 4) switch positions, 4
# channels, and two values for each channel.
COMP_MAT_SIZE = 8 * 4 * 2

def Conditioning():
    SC_IQ_LENGTH = Parameter('SC_IQ_LENGTH', 'Length of SC waveform')

    SetChannelName('SC')

    # Configuration values
    aOut('MAXDEV', 0, 100,
        PREC = 1, EGU  = '%',
        DESC = 'Maximum signal deviation (%)')
    aOut('CIIR', 0, 1,
        PREC = 2,
        DESC = 'IIR factor for channels')
    aOut('INTERVAL', 0, 100,
        ESLO = 1e-3, PREC = 1, EGU  = 's',
        DESC = 'Conditioning interval')
    boolOut('TRIGGERED', 'Free Run', 'Triggered',
        DESC = 'Triggered or free running')
    longOut('TRIGDELAY',
        DESC = 'Conditioning trigger delay')

    # Available for external direct control of the conditioning matrix.
    Waveform('SETCOMP_S', 16*4*2, DESC = 'Low level compensation cntrl')

    # Readback values
    Trigger(False, [
            mbbIn('STATUS',
                ('Off',             0, 'NO_ALARM'),
                ('Data read error', 1, 'MAJOR'),
                ('No Switch',       2, 'MINOR'),
                ('High variance',   3, 'MINOR'),
                ('Overflow',        4, 'MAJOR'),
                ('Good',            5, 'NO_ALARM'),
                DESC = 'Signal conditioning status'),
            aIn('DEV', 0, 200,
                PREC = 1, EGU  = '%',
                DESC = 'Relative signal deviation'),

            # Digest of the IQ_wf() data reduced to one complex value per
            # button and switch position pair.
            Waveform('IQDIGEST', 8*4*2, FTVL = 'DOUBLE',
                DESC = 'Raw digest of IQ data'),
            # Compensation matrix used to generate the IQ_wf() array
            Waveform('LASTCOMP', COMP_MAT_SIZE,
                DESC = 'Last channel compensation'),
            # Current compensation matrix: will be copied to LASTCOMP the
            # next time SC processes.
            Waveform('COMP', COMP_MAT_SIZE,
                DESC = 'Current channel compensation'),
        ] + [
            aIn('PHASE%s' % button, -180, 180,
                PREC = 3, EGU  = 'deg',
                DESC = 'Button %s input phase' % button)
            for button in 'BCD'
        ] + [
            pv
            for channel in range(1,5)
            for pv in
                aIn('C%sPHASE' % channel, -180, 180,
                    PREC = 3, EGU  = 'deg',
                    DESC = 'Channel %s phase shift' % channel),
                aIn('C%sMAG' % channel, 0.8, 1.2,
                    PREC = 5,
                    DESC = 'Channel %s gain' % channel),
                aIn('C%sVAR' % channel, 0, 0.1,
                    PREC = 4,
                    DESC = 'Channel %s variance' % channel),
        ] + IQ_wf(SC_IQ_LENGTH))

    UnsetChannelName()


# -----------------------------------------------------------------------------



def AggregateSeverity(name, description, recs):
    '''Aggregates the severity of all the given records into a single record.
    The value of the record is constant, but its SEVR value reflects the
    maximum severity of all of the given records.'''

    assert len(recs) <= 12, 'Too many records to aggregate'
    return records.calc(name,
        CALC = 1, DESC = description,
        # Assign each record of interest to a calc record input with MS.
        # This then automatically propagates to the severity of the whole
        # record.
        **dict([
            ('INP' + c, MS(r))
            for c, r in zip ('ABCDEFGHIJKL', recs)]))


def ClockStatus(id, name):
    health = [
        # MC monitoring PVS
        mbbIn('%s_LOCK' % id,
            ('No Clock',        0, 'MAJOR'),
            ('Seek Frequency',  1, 'MAJOR'),
            ('Slewing',         2, 'MINOR'),
            ('Phase Locked',    3, 'NO_ALARM'),
            ('Open Loop',       4, 'MAJOR'),
            DESC = '%s clock lock status' % name),
        mbbIn('%s_SYNC' % id,
            ('Not Synched',     0, 'MINOR'),
            ('Waiting Trigger', 1, 'MINOR'),
            ('Synchronised',    2, 'NO_ALARM'),
            DESC = '%s clock sync state' % name)]
    Trigger(False, health,
        TRIG = '%s_S_TRIG' % id, DONE = '%s_S_DONE' % id)

    detail = [
        longIn('%s_DAC' % id, 0, 65535,
            DESC = '%s clock DAC setting' % name),
        longIn('%s_PHASE_E' % id,
            DESC = '%s clock phase error' % name),
        longIn('%s_FREQ_E' % id,
            DESC = '%s clock freq error' % name)]
    Trigger(False, detail,
        TRIG = '%s_V_TRIG' % id, DONE = '%s_V_DONE' % id)

    return health



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

    boolOut('VERBOSE', 'Silent', 'Monitor',
        ZSV  = 'MINOR',     OSV  = 'NO_ALARM',
        DESC = 'Enable clock monitoring')

    # Direct open loop DAC control
    boolOut('OPEN_LOOP', 'PLL Control', 'Open Loop',
        ZSV  = 'NO_ALARM',  OSV  = 'MAJOR',
        DESC = 'Enable phased locked loop')
    longOut('MC_DAC', 0, 65535, DESC = 'Open loop MC DAC control')
    longOut('SC_DAC', 0, 65535, DESC = 'Open loop SC DAC control')

    # Create the tick health monitor.  This records the number of seconds
    # since the last recorded trigger and signals an alarm according to how
    # long the delay has been.
    tick = records.calc('TICK',
        SCAN = '.5 second',
        CALC = 'A+0.5',
        EGU  = 's', PREC = 1,
        HIGH = 1,   HSV  = 'MINOR')
    tick.INPA = tick
    tick_reset = records.calcout('TICK_CALC',
        CALC = '0',
        OUT  = tick,
        OOPT = 'Every Time',
        DOPT = 'Use CALC')

    # Records for calculating number of missed triggers and associated
    # percentage.
    missed = longIn('MISSED', DESC = 'Triggers missed since last')
    tick_count = records.calc('TICK_COUNT',
        CALC = 'A+1',   DESC = 'Total triggers seen')
    tick_count.INPA = tick_count
    total_missed = records.calc('MISSED_ALL',
        CALC = 'A+B',   INPA = missed,
        DESC = 'Accumulated triggers missed')
    total_missed.INPB = total_missed
    percent_missed = records.calc('MISSED_PC',
        CALC = 'A>0?100*A/(A+B):0',
        INPA = total_missed,    INPB = tick_count,
        EGU  = '%',     PREC = 2,
        DESC = 'Percentage triggers missed')
    # A reset record to simultaneously reset both counters
    create_dfanout('RESET_CNTRS',
        tick_count, total_missed,
        VAL = 0,    PINI = 'YES',   DESC = 'Reset tick loss counters')

    # This trigger receives the normal machine trigger.
    Trigger(True, [
            tick_reset, tick_count,
            missed, total_missed, percent_missed,
            stringIn('TIME_NTP', DESC = 'NTP time'),
            stringIn('TIME_SC',  DESC = 'System clock time')],
        TRIG = 'TIME', DONE = 'TIME_DONE')

    # Generate the clock monitoring records.
    mc_health = ClockStatus('MC', 'Machine')
    sc_health = ClockStatus('SC', 'System')
    sc_health = []  # For the time being don't monitor SC health

    # The following list must match the corresponding enum in sensors.cpp
    ntp_health = mbbIn('NTPSTAT',
        ('Not monitored',   0,  'NO_ALARM'),    # Monitoring disabled
        ('No NTP server',   1,  'MAJOR'),       # Local NTP server not found
        ('Startup',         2,  'NO_ALARM'),    # No alarm during startup
        ('No Sync',         3,  'MINOR'),       # NTP server not synchronised
        ('Synchronised',    4,  'NO_ALARM'),    # Synchronised to remote server
        DESC = 'Status of NTP server')
    global ntp_monitors                 # Needed for sensor record triggering
    ntp_monitors = [ntp_health,
        longIn('STRATUM',
            LOW  = 0,   LSV  = 'MAJOR',         # Probably does not occur now
            HIGH = 16,  HSV  = 'MAJOR',         # Unspecified stratum
            DESC = 'NTP stratum level'),
        stringIn('SERVER', DESC = 'Synchronised NTP server')]


    clock_health = AggregateSeverity('HEALTH', 'Clock status',
        map(CP, mc_health + sc_health + [ntp_health, tick]))

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
        VoltageSensor(1, 'Virtex core power supply',    1.46),
        VoltageSensor(2, '(Unused voltage)',            2.47, False),
        VoltageSensor(3, 'Virtex ADC interface',        2.44),
        VoltageSensor(4, 'SBC & SDRAM power supply',    3.20),
        VoltageSensor(5, 'PMC +5V power supply',        4.94),
        VoltageSensor(6, 'Fans power supply',           11.9),
        VoltageSensor(7, 'PMC -12V power supply',       -12.2),
        VoltageSensor(8, 'Attenuators & switches ctrl', -5.09)]


    health = AggregateSeverity('VOLTSOK', 'Voltage health', voltages)
    health_display = records.mbbi('VHEALTH',
        DESC = 'Voltage health display',
        INP  = MS(health.SEVR),
        ZRVL = 0,   ZRST = 'Ok',
        ONVL = 1,   ONST = 'Warning',
        TWVL = 2,   TWST = 'Fault')

    return voltages + [health, health_display], health


# Extra health records can be accumulated onto this list
ExtraHealthRecords = []

def Sensors():
    SetChannelName('SE')

    enable = mbbOut('HEALTHD',
        ('Healthd On',      0, 'NO_ALARM'),
        ('Healthd Off',     1, 'NO_ALARM'),
        ('Healthd Silent',  2, 'MAJOR'),
        DESC = 'Sensor monitoring control')
    longOut('SETTEMP', EGU = 'deg C',
        DESC = 'Health daemon target temp')

    # The alarm settings for the temperatures track how close we are to the
    # panic temperature.  For RF1 we panic (by default) at 80 degrees, and for
    # MB we panic at 70; RF2 isn't monitored for panic detection, but it
    # typically runs 5 degrees hotter than RF1.  We set the alarm temperatures
    # 10 and 15 degrees below the default panic temperatures.
    temp_monitors = [
        longIn('TEMP', 30, 70, 'deg C',
            DESC = 'Monitored board temperature'),
        longIn('TEMP_RF1', 40, 70, 'deg C',
            DESC = 'RF board temperature',
            HIGH = 65,    HSV  = 'MINOR',
            HIHI = 70,    HHSV = 'MAJOR'),
        aIn('TEMP_RF2', 40, 70, 1e-3, 'deg C',
            DESC = 'RF board temperature',
            PREC = 1,
            HIGH = 70,    HSV  = 'MINOR',
            HIHI = 75,    HHSV = 'MAJOR'),
        longIn('TEMP_MB', 30, 60, 'deg C',
            DESC = 'Motherboard temperature',
            HIGH = 55,    HSV  = 'MINOR',
            HIHI = 60,    HHSV = 'MAJOR')]

    fan_health = []
    for i in (1, 2):
        fan_speed = longIn('FAN%d' % i,     4000, 6000, 'RPM',
            DESC = 'Fan %d speed' % i)
        fan_set = longIn('FAN%d_SET' % i,   4000, 6000, 'RPM',
            DESC = 'Fan %d set speed' % i)
        fan_err = records.calc('FAN%d_ERR' % i,
            DESC = 'Fan %d speed error' % i,
            CALC = 'A-B',   INPA = fan_speed,   INPB = fan_set,
            EGU  = 'RPM',
            LOLO = -1000,   LLSV = 'MAJOR', LOW  = -500,    LSV  = 'MINOR',
            HIGH = 500,     HSV  = 'MINOR', HIHI = 1000,    HHSV = 'MAJOR')
        fan_health.append(fan_err)
        temp_monitors.extend([fan_speed, fan_set, fan_err])
    fan_health = AggregateSeverity('FAN:OK',
        'Fan controller health', fan_health)

    alarmsensors = [
        aIn('FREE', 0, 64, 1./MB, 'MB', 2,
            DESC = 'Free memory',
            LOW  = 12,      LSV  = 'MINOR',
            LOLO = 8,       LLSV = 'MAJOR'),
        aIn('RAMFS', 0, 64, 1./MB, 'MB', 3,
            DESC = 'Temporary file usage',
            HIGH = 1,       HSV  = 'MINOR',
            HIHI = 16,      HHSV = 'MAJOR'),
        aIn('CPU', 0, 100, 1e-3, '%', 1,
            DESC = 'CPU usage',
            HIGH = 80,      HSV  = 'MINOR',
            HIHI = 95,      HHSV = 'MAJOR')]

    voltages, voltage_health = Voltages()
    alarmsensors.append(voltage_health)

    extras = [
        # Time since booting
        aIn('UPTIME', 0, 24*3600*5, 1./3600, 'h', 2,
            DESC = 'Total system up time'),
        aIn('EPICSUP', 0, 24*3600*5, 1./3600, 'h', 2,
            DESC = 'Time since EPICS started'),

        # Channel access counters
        longIn('CAPVS',  DESC = 'Number of connected PVs'),
        longIn('CACLNT', DESC = 'Number of connected clients'),

        # Network statistics
        aIn('NWBRX', 0, 1e4, 1e-4, 'kB/s', 3,
            DESC = 'Kilobytes received per second'),
        aIn('NWBTX', 0, 1e4, 1e-4, 'kB/s', 3,
            DESC = 'Kilobytes sent per second'),
        aIn('NWPRX', 0, 1e4, 0.1, 'pkt/s', 1,
            DESC = 'Packets received per second'),
        aIn('NWPTX', 0, 1e4, 0.1, 'pkt/s', 1,
            DESC = 'Packets sent per second'),
        aIn('NWMRX', 0, 1e4, 0.1, 'pkt/s', 1,
            DESC = 'Multicast received per second'),
        aIn('NWMTX', 0, 1e4, 0.1, 'pkt/s', 1,
            DESC = 'Multicast sent per second')]

    # Aggregate all the alarm generating records into a single "health"
    # record.  Only the alarm status of this record is meaningful.
    temp_health = AggregateSeverity(
        'TEMPMON', 'Temperature monitoring', temp_monitors + [enable])
    all_health = AggregateSeverity('HEALTH', 'Aggregated health',
        alarmsensors + ExtraHealthRecords + [temp_health])

    Trigger(False,
        temp_monitors + alarmsensors + voltages + ntp_monitors + extras +
        [fan_health, temp_health, all_health])

    UnsetChannelName()

    # Mirror the health record for backwards compatibility.
    records.bi('HEALTH', INP = MS(CP(all_health)))


def Versions():
    def string(name, description):
        return stringIn(name, PINI = 'YES', DESC = description)
    def longin(name, description):
        return longIn(name, PINI = 'YES', DESC = description)
    def boolin(name, description):
        return boolIn(name, 'No', 'Yes',
            PINI = 'YES', DESC = description)

    string('VERSION',   'Libera EPICS driver version')
    string('BUILD',     'EPICS driver build date')

    boolOut('REBOOT',  'Reboot',  DESC = 'Reboot Libera IOC')
    boolOut('RESTART', 'Restart', DESC = 'Restart EPICS driver')

    SetChannelName('VE')

    # These are internally generated by Libera
    string('VERSION',   'Libera EPICS driver version')
    string('BUILD',     'EPICS driver build date')
    string('EPICS',     'EPICS version')
    string('COMPILER',  'Compiler version')
    string('LIBRARY',   'Linked library version')

    # These are extracted from environment variables
    string('ABI',       'ABI identification (3=>EABI)')
    string('UNAME',     'Kernel version')
    string('LIBC',      'Libc version')
    string('DRIVER',    'Libera driver version')
    string('MSP',       'MSP driver version')
    string('FPGA',      'FPGA version')
    string('ARCH',      'Architecture identification')
    string('ROOTFS',    'SBC rootfs distribution')

    # These are all FPGA registers
    longin('COMPILED',  r'FPGA \"compiled\" register') # escape in builder!
    longin('BUILDNO',   'FPGA build number register')
    longin('CUSTID',    'FPGA customer id register')
    longin('DDCDEC',    'Libera samples per turn')
    longin('FACIC',     'FA CIC decimation factor')
    longin('FAFIR',     'FA FIR decimation factor')
    longin('FADEC',     'Turns per FA update')
    longin('CUSTOMER',  'Customer feature register')
    longin('ITECH',     'i-Tech feature register')
    # Customer Id as a string
    string('CUSTIDSTR', 'FPGA customer id')

    boolin('BR',        'Libera Brillance FPGA')
    boolin('BRHW',      'Brilliance hardware detected')
    boolin('OLDBR',     'Old Brilliance attenuators')
    boolin('DLS',       'DLS FGPA')
    boolin('FF',        'Fast Feedback enabled')
    boolin('GBETH',     'Gigabit FF controller')
    boolin('MAF',       'Boxcard filter present')
    boolin('ITMAXADC',  'i-Tech MAX ADC register')
    boolin('FPGA2',     'Libera 2.00+ FPGA features')
    boolin('ILK2',      'Secondary interlock control')
    boolin('FAPAY',     'FA payload selection')
    boolin('DRIVER2',   'Libera 2.00+ driver features')

    UnsetChannelName()



# Finally generate and output the supported records.

FirstTurn()         # FT - one position from somewhere within a 100us window
Booster()           # BN - 1024:1 (and 64:1) decimated waveforms
FreeRunning()       # FR - turn by turn, updating on every trigger
TurnByTurn()        # TT - very long waveforms, triggered once on demand
SlowAcquisition()   # SA - 10Hz low noise position
Postmortem()        # PM - fixed length pre-postmortem trigger waveforms

Config()            # CF - general configuration records
Interlock()         # IL - interlock configuration records
Conditioning()      # SC - signal conditioning records
Clock()             # CK - clock monitoring and control
Sensors()           # SE - temperatures, fan speeds, memory and CPU usage etc
Versions()          # VE - version information records, and miscellaneous

WriteRecords(sys.argv[1])
