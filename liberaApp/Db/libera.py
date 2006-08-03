# This file is part of the Libera EPICS Driver,
# Copyright (C) 2005  Michael Abbott, Diamond Light Source Ltd.
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
from support import Libera
from epics import *



# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------


def extend(dictionary, key, value):
    assert key not in dictionary, 'key %s already in dictionary' % key
    dictionary[key] = value


# Functions for creating output records

def aOut(name, DRVL, DRVH, ESLO=1e-6, EOFF=0, PREC=4, **fields):
    Libera.ao(name + '_S', address=name,
        OMSL = 'supervisory', 
        ESLO = ESLO,  EOFF = EOFF,  LINR = 'LINEAR',
        PREC = PREC,  
        DRVL = DRVL,  DRVH = DRVH,
        EGUL = DRVL,  EGUF = DRVH,
        **fields)

def boolOut(name, ZNAM, ONAM, **fields):
    Libera.bo(name + '_S', address=name,
        OMSL = 'supervisory', 
        ZNAM = ZNAM, ONAM = ONAM, **fields)

def longOut(name, LOPR, HOPR, **fields):
    Libera.longout(name + '_S', address=name,
        OMSL = 'supervisory',
        LOPR = LOPR, HOPR = HOPR, **fields)

def mbbOut(name, *option_values, **fields):
    mbbPrefixes = [
        'ZR', 'ON', 'TW', 'TH', 'FR', 'FV', 'SX', 'SV',     # 0-7
        'EI', 'NI', 'TE', 'EL', 'TV', 'TT', 'FT', 'FF']     # 8-15
    for prefix, (option, value) in zip(mbbPrefixes, option_values):
        extend(fields, prefix + 'ST', option)
        extend(fields, prefix + 'VL', value)
    Libera.mbbo(name + '_S', address=name,
        OMSL = 'supervisory', **fields)
    

def Waveform(name, length, FTVL='LONG', **fields):
    return Libera.waveform(
        name, name,
        SCAN = 'Passive',
        NELM = length,
        FTVL = FTVL,
        **fields)


# Boilerplate record generation.  Here is where the various types of records
# that can be generated are defined.

MAX_INT = 2**31 - 1
MAX_nm  = 10 * 10**6         # 10^7 nm = 10 mm
MAX_mm  = 1e-6 * MAX_nm
MAX_S   = MAX_INT

def RAW_ADC(length):
    return [
        Waveform('RAW' + button, length,
            LOPR = -2048, HOPR = 2047)
        for button in 'ABCD']

def IQ_wf(length):
    return [
        Waveform('WF' + button + axis, length,
            LOPR = -MAX_S, HOPR = MAX_S)
        for button in 'ABCD' for axis in 'IQ']

def ABCD_wf(length):
    return [
        Waveform('WF' + button, length,
            LOPR = 0, HOPR = MAX_S)
        for button in 'ABCD']

def ABCD_():
    return [
        Libera.longin(button,
            MDEL = -1,
            LOPR = 0, HOPR = MAX_S)
        for button in 'ABCD']

def XYQS_wf(length, prefix='WF'):
    return [
        Waveform(prefix + position, length,
            LOPR = -MAX_nm, HOPR = MAX_nm, EGU = 'nm')
        for position in 'XYQ'] + [
        Waveform(prefix + 'S', length,
            LOPR = 0, HOPR = MAX_S)]

def XYQS_(prec, logMax=0, suffix=''):
    sl = [Libera.longin('S' + suffix, MDEL = -1)]
    if logMax:
        sl.append(records.calc('SL' + suffix,
            CALC = '20*(LOG(A)-%g)' % logMax,
            INPA = sl[0],
            MDEL = -1,
            LOPR = -50,   HOPR = 0,
            EGU  = 'dB',  PREC = 0))
    return [
        Libera.ai(position + suffix,
            MDEL = -1,    LINR = 'LINEAR',
            EOFF = 0,     ESLO = 1e-6,
            LOPR = 0,     HOPR = MAX_mm,
            PREC = prec,  EGU  = 'mm')
        for position in 'XYQ'] + sl
        
        

def Enable():
    boolOut('ENABLE', 'Disabled', 'Enabled')

def Trigger(*positions):
    # The DONE record must be processed after all other triggered records are
    # processed: this is used as an interlock to synchronise with the Libera
    # driver.
    done = Libera.bo('DONE')
    Libera.bi('TRIG',
        SCAN = 'I/O Intr',
        FLNK = create_fanout('FAN', *positions + (done,)))

        
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
    
    maxadc = Libera.longin('MAXADC',
        LOPR = 0,        HOPR = 2048,
        HSV  = 'MINOR',  HIGH = 1024,                   # -6dB
        HHSV = 'MAJOR',  HIHI = int(1024*sqrt(2)))      # -3dB

    Trigger(*
        # Raw wavefors as read from the ADC rate buffer
        RAW_ADC(LONG_LENGTH) + [maxadc] +
        # ADC data reduced by 1/4 by recombination
        ABCD_wf(SHORT_LENGTH) +
        # Synthesised button positions from windowed averages
        ABCD_() + 
        # Final computed positions with logarithmic scale.
        XYQS_(2, logMax = log10(2**30 * 0.582217 * sqrt(2))))

    # Sample window control
    longOut('OFF', 0, SHORT_LENGTH - 1)
    longOut('LEN', 1, SHORT_LENGTH)

    UnsetChannelName()


# Booster ramp support records.  Decimated waveforms for overview of entire
# 100ms booster ramp.
def Booster():
    SHORT_LENGTH = Parameter('BN_SHORT')    # 190  = BN_LONG / 16
    LONG_LENGTH  = Parameter('BN_LONG')     # 3040 = BN_SHORT * 16

    SetChannelName('BN')
    Enable()
    
    Trigger(*
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
    Waveform('AXIS',  LONG_LENGTH,  'FLOAT', PINI='YES')
    Waveform('AXISS', SHORT_LENGTH, 'FLOAT', PINI='YES')

    UnsetChannelName()


# Free running short (typically 2048) turn-by-turn buffer.  
def FreeRunning():
    LENGTH = Parameter('FR_LENGTH')
    
    SetChannelName('FR')
    Enable()

    # In this mode we provide all the available data: raw IQ, buttons and
    # computed positions.
    Trigger(*IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH))
    
    UnsetChannelName()
        

# Postmortem fixed length buffer.  This mode is always enabled.
def Postmortem():
    LENGTH = 16384
    
    SetChannelName('PM')

    # All turn-by-turn data is provided.
    Trigger(*IQ_wf(LENGTH) + ABCD_wf(LENGTH) + XYQS_wf(LENGTH))
    
    UnsetChannelName()
        

# Turn-by-turn snapshot records.  Access to long waveforms captured on
# request.  Typically used for tune measurements.  Up to 200,000 points can
# be captured in one request.
def TurnByTurn():
    LONG_LENGTH   = Parameter('TT_LONG')
    WINDOW_LENGTH = Parameter('TT_WINDOW')
    
    SetChannelName('TT')
    
    # Number of points successfully captured by the last trigger.
    captured = Libera.longin('CAPTURED')
    # Write a one to request a refill of the buffer.
    arm = Libera.bo('ARM',
        ZNAM = 'Not armed',
        ONAM = 'Trigger enabled')
    # This indicates whether the buffer has been filled.
    ready = Libera.bi('READY',
        SCAN = 'I/O Intr',
        PINI = 'YES',
        ZNAM = 'No waveform',
        ONAM = 'Waveform ready')
    # When the ready record makes a transition into the ready state re-enable
    # the arm record by setting it to zero.
    rearm = records.calcout(
        'REARM',
        OUT  = arm,
        INPA = ready,
        CALC = '!A',
        OOPT = 'Transition To Zero',
        DOPT = 'Use CALC')
    ready.FLNK = create_fanout('FANA', rearm, captured)

    # Readout window and total capture length
    longOut('LENGTH', 1, WINDOW_LENGTH)
    longOut('CAPLEN', 1, LONG_LENGTH)
    # Readout window offset together with a readback to be used as a readout
    # interlock.
    longOut('OFFSET', 0, LONG_LENGTH)
    offset = Libera.longin('OFFSET', PINI = 'YES')

    Trigger(*
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
    CurrentEGU  = 'mA'
    CurrentESLO = 1e-5
    
    SetChannelName('SA')
    power = Libera.ai('POWER',
        MDEL = -1,  LINR = 'LINEAR',
        EOFF = 0,   ESLO = 1e-6,
        LOPR = -80, HOPR = 10,
        PREC = 3,   EGU  = 'dBm')
    current = Libera.ai('CURRENT',
        MDEL = -1,  LINR = 'LINEAR',
        EOFF = 0,   ESLO = CurrentESLO,
        LOPR = 0,   HOPR = 500,
        PREC = 3,   EGU  = CurrentEGU)
    Trigger(*ABCD_() + XYQS_(4) + XYQS_(4, suffix='C') + [power, current])
    aOut('ISCALE', 0, 20000,
        EGU  = CurrentEGU,
        ESLO = CurrentESLO,
        PREC = 1)
    UnsetChannelName()



# ----------------------------------------------------------------------------
#           Configuration
# ----------------------------------------------------------------------------
        

# Configuration control records.  Used for setting button or stripline
# geometry and 
def Config():
    SetChannelName('CF')
    boolOut('DIAG', 'VERTICAL', 'DIAGONAL')
    # Geometry calibration control
    aOut('KX', 0, 32,   EGU  = 'mm')
    aOut('KY', 0, 32,   EGU  = 'mm')
    aOut('KQ', 0, 32,   EGU  = 'mm')
    aOut('X0', -16, 16, EGU  = 'mm')
    aOut('Y0', -16, 16, EGU  = 'mm')
    # Channel gain settings.  Only applies to first turn mode.
    aOut('G0', 0, 1.5,  ESLO = 2**-30)
    aOut('G1', 0, 1.5,  ESLO = 2**-30)
    aOut('G2', 0, 1.5,  ESLO = 2**-30)
    aOut('G3', 0, 1.5,  ESLO = 2**-30)

    # Switch automatic switching on or off
    boolOut('AUTOSW', 'Fixed', 'Automatic')
    # Select switch to use when automatic switching off
    longOut('SETSW', 0, 15)
    # Switch automatic gain control on or off
    boolOut('AGC', 'Manual', 'Automatic')
    # Select attenuation to use when AGC is off
    longOut('SETATTEN', 0, 62, EGU = 'dB')
    # Currently configured attenuation
    Libera.longin('ATTEN',
        SCAN = '1 second', LOPR = 0, HOPR = 62, EGU = 'dB')

    UnsetChannelName()


# Interlock control records.  Used for configuring interlock operation.
def Interlock():
    SetChannelName('IL')

    # Interlock window limits in mm
    aOut('MINX', -5, 5, EGU = 'mm')
    aOut('MAXX', -5, 5, EGU = 'mm')
    aOut('MINY', -5, 5, EGU = 'mm')
    aOut('MAXY', -5, 5, EGU = 'mm')

    # Interlock gain limits
    longOut('GAIN', -62, 0, EGU = 'dB')

    # Overflow detection and limits
    longOut('OVER', 0, 2048)
    longOut('TIME', 0, 1000)

    # Interlock mode: disabled, enabled or gain dependent.
    mbbOut('MODE', ('Disable', 0), ('Enable', 1), ('Gain Dependent', 3))

    # Interlock state.  This is a bit nasty: we get repeated triggers on TRIG
    # while the interlock is active (ie, reporting signal bad).  The records
    # POKE_STATE simply acts to relay the trigger state to STATE, which
    # automatically resets itself if no triggers are received.
    trigger = Libera.bi('TRIG', 
        SCAN = 'I/O Intr',
        VAL  = 1,  PINI = 'YES',
        ONAM = 'Trigger')
    state = records.bo('STATE',
        HIGH = 0.5,
        OMSL = 'supervisory',
        ZNAM = 'Ready',        ZSV  = 'NO_ALARM',
        ONAM = 'Interlocked',  OSV  = 'MAJOR')
    trigger.FLNK = records.bo('POKE_STATE',
        DOL  = trigger,  OMSL = 'closed_loop',
        OUT  = PP(state))
    
    UnsetChannelName()

    
def Miscellaneous():
    # The VERSION and BUILD strings are not tied to any "channel".
    Libera.stringin('VERSION', PINI = 'YES')
    Libera.stringin('BUILD',   PINI = 'YES')

    # Similarly the tick health monitor is defined here.  This records the
    # number of seconds since the last recorded trigger and signals an alarm
    # according to how long the delay has been.
    tick = records.calc('TICK',
        SCAN = '.1 second',
        CALC = 'A+0.1',
        EGU  = 's', PREC = 1,
        HIGH = 1,   HSV  = 'MINOR',
        HIHI = 10,  HHSV = 'MAJOR')
    tick.INPA = tick
    tick_reset = records.calcout('TICK_CALC',
        CALC = '0',
        OUT  = tick,
        OOPT = 'Every Time',
        DOPT = 'Use CALC')
    Libera.bi('TICK_TRIG', SCAN = 'I/O Intr', FLNK = tick_reset)

    

# Finally generate and output the supported records.
    
FirstTurn()         # FT - one position from somewhere within a 100us window
Booster()           # BN - 1024:1 (and 64:1) decimated waveforms
FreeRunning()       # FR - turn by turn, updating on every trigger
TurnByTurn()        # TT - very long waveforms, triggered once on demand
SlowAcquisition()   # SA - 10Hz low noise position
Postmortem()        # PM - fixed length pre-postmortem trigger waveforms

Config()            # CF - general configuration records
Interlock()         # IL - interlock configuration records
Miscellaneous()     # Other records
WriteRecords(sys.argv[1])
