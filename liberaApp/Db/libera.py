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
# module initialises epics (and determined which symbols it exports to *!)
from support import Libera
from epics import *



# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------
        

# Functions for creating bound pairs of set/readback records.

def aInOut(name, DRVL, DRVH, **fields):
    input  = Libera.ai(name,
        PINI = 'YES',
        EGUL = DRVL,  EGUF = DRVH,
        **fields)
    output = Libera.ao(name + '_S', address=name,
        FLNK = input,
        OMSL = 'supervisory', LINR = 'NO CONVERSION',
        DRVL = DRVL,  DRVH = DRVH,
        EGUL = DRVL,  EGUF = DRVH,
        **fields)
    return input, output

def boolInOut(name, **fields):
    input = Libera.bi(name, PINI = 'YES', **fields)
    output = Libera.bo(name + '_S', address=name, FLNK = input, **fields)
    return input, output

def longInOut(name, OMSL='supervisory', **fields):
    input  = Libera.longin(name, PINI = 'YES', **fields)
    output = Libera.longout(name + '_S', address=name,
        FLNK = input, OMSL = 'supervisory', **fields)
    return input, output

    
def Waveform(name, length, FTVL='LONG', **fields):
    return Libera.waveform(
        name, name,
        SCAN = 'Passive',
        NELM = length,
        FTVL = FTVL,
        **fields)


# Boilerplate record generation.  Here is where the various types of records
# that can be generated are defined.
# 
# ChannelName = None
# def SetChannelName(name):
#     global ChannelName
#     ChannelName = name
# def Name(button):
#     return '%s:%s' % (ChannelName, button)

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

def XYQS_(prec=1, logMax=0):
    sl = [Libera.longin('S', MDEL = -1)]
    if logMax:
        sl.append(records.calc('SL',
            CALC = '20*(LOG(A)-%g)' % logMax,
            INPA = sl[0],
            MDEL = -1,
            LOPR = -50,   HOPR = 0,
            EGU  = 'dB',  PREC = 0))
    return [
        Libera.ai(position,
            MDEL = -1,
            LOPR = 0,     HOPR = MAX_mm,
            PREC = prec,  EGU  = 'mm')
        for position in 'XYQ'] + sl
        
        

def Enable():
    return      # Not yet implemented
    boolInOut('ENABLE',
        ZNAM = 'Disabled',
        ONAM = 'Enabled')

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
        XYQS_(logMax = log10(2**30 * 0.582217 * sqrt(2))))

    # Sample window control
    longInOut('OFF', LOPR = 0, HOPR = SHORT_LENGTH - 1)
    longInOut('LEN', LOPR = 1, HOPR = SHORT_LENGTH)

    UnsetChannelName()


# Booster ramp support records.  Decimated waveforms for overview of entire
# 100ms booster ramp.
def Booster():
    SHORT_LENGTH = Parameter('BN_SHORT')    # 190  = BN_LONG / 16
    LONG_LENGTH  = Parameter('BN_LONG')     # 3040 = BN_SHORT * 16

    SetChannelName('BN')
    Enable()
    
    Trigger(*
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
    
    SetChannelName('TT')    # <=== Soon to be split into TT and LT
    Enable()
    
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
    longInOut('LENGTH', LOPR = 1, HOPR = WINDOW_LENGTH)
    longInOut('CAPLEN', LOPR = 1, HOPR = LONG_LENGTH)
    # Readout window offset.  Here we break the usual chain where OFFSET
    # would update as soon as OFFSET_S is written: instead, offset won't
    # update until all the waveforms have been processed.  This allows OFFSET
    # to be used as a readout interlock.
    Libera.longout('OFFSET_S', 'OFFSET',
        OMSL = 'supervisory', LOPR = 0, HOPR = LONG_LENGTH)
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
    SetChannelName('SA')
    Enable()
    Trigger(*ABCD_() + XYQS_())
    UnsetChannelName()



# ----------------------------------------------------------------------------
#           Configuration
# ----------------------------------------------------------------------------
        

# Configuration control records.  Used for setting button or stripline
# geometry and 
def Config():
    SetChannelName('CF')
    boolInOut('DIAG', ZNAM = 'VERTICAL', ONAM = 'DIAGONAL')
    aInOut('KX', 0, 32,   PREC = 4, EGU  = 'mm')
    aInOut('KY', 0, 32,   PREC = 4, EGU  = 'mm')
    aInOut('KQ', 0, 32,   PREC = 4, EGU  = 'mm')
    aInOut('X0', -16, 16, PREC = 4, EGU  = 'mm')
    aInOut('Y0', -16, 16, PREC = 4, EGU  = 'mm')
    aInOut('GA', 0, 1.5,  VAL  = 1, PREC = 4)
    aInOut('GB', 0, 1.5,  VAL  = 1, PREC = 4)
    aInOut('GC', 0, 1.5,  VAL  = 1, PREC = 4)
    aInOut('GD', 0, 1.5,  VAL  = 1, PREC = 4)

    attenwf = Libera.waveform('ATTWF',
        NELM = 8, FTVL = 'LONG', PINI = 'YES')
    Libera.longout('ATT1_S', 'ATT1',
        OMSL = 'supervisory', LOPR = 0, HOPR = 31, FLNK = attenwf)
    Libera.longout('ATT2_S', 'ATT2',
        OMSL = 'supervisory', LOPR = 0, HOPR = 31, FLNK = attenwf)

    longInOut('SW', LOPR = 0, HOPR = 15)
    UnsetChannelName()


# Not yet used ...
#   These interfaces will probably change drastically.

def Fast():
    SetChannelName('FF')
    waveforms = [
        Libera.waveform(
            'FAST:WF' + name,
            FTVL = 'LONG',
            NELM = 4000)
        for name in 'XYSQ']

    trigger = Libera.bi(
        'FAST:TRIG',
        SCAN = 'I/O Intr',
        DESC = 'Fast feedback trigger')
    trigger.FLNK = records.create_fanout('FAST:FAN', waveforms)
    UnsetChannelName()



# Finally generate and output the supported records.
    
FirstTurn()
Booster()
FreeRunning()
TurnByTurn()
SlowAcquisition()
Postmortem()

Config()

WriteRecords(sys.argv[1])
