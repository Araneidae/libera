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
import os
from math import *

import epics
epics.Configure(recordnames=epics.TemplateRecordNames())
 
from epics import *

# Ensure that we can find the Libera dbd files.  The home directory is where
# the top level Libera directory has been placed.
HomeDir = os.path.realpath(
    os.path.join(os.path.dirname(sys.argv[0]), '../../..'))
LibVersion('Libera', home=HomeDir)




# This class wraps the creation of records which talk directly to the
# Libera device driver.
class Libera(hardware.Device):
    @classmethod
    def LoadLibrary(cls):
        cls.LoadDbdFile('dbd/libera.dbd')

    class makeRecord:
        def __init__(self, builder, addr_name):
            self.builder = getattr(records, builder)
            self.addr_name = addr_name

        def __call__(self, name, address=None, **fields):
            if address is None:
                address = name
            record = self.builder(name, **fields)
            if not GetSimulation():
                record.DTYP = 'Libera'
                setattr(record, self.addr_name, address)
            return record

    @classmethod
    def init(cls):
        for name, addr in [
                ('longin',   'INP'), ('longout',  'OUT'),
                ('ai',       'INP'), ('ao',       'OUT'),
                ('bi',       'INP'), ('bo',       'OUT'),
                ('waveform', 'INP')]:
            setattr(cls, name, cls.makeRecord(name, addr))

Libera.init()
Libera()



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

        

# First turn snapshot records.  Access to position data immediately following
# the trigger for use on transfer paths and during injection.
def FirstTurn():
    waveforms = [
        Waveform('FT:WF%s' % button, 256)
        for button in 'ABCD'] + [
        Waveform('FT:RAW%s' % button, 1024)
        for button in 'ABCD']

    position_S = Libera.longin('FT:S')
    positions = \
        [position_S] + \
        [Libera.longin('FT:%s' % position) for position in 'ABCD'] + \
        [Libera.ai('FT:%s' % position, PREC = 1, EGU  = 'mm')
            for position in 'XYQ'] + \
        [Libera.longin('FT:MAXADC',
            LOPR = 0,        HOPR = 2048,
            HSV  = 'MINOR',  HIGH = 1024,                   # -6dB
            HHSV = 'MAJOR',  HIHI = int(1024*sqrt(2)), )]   # -3dB

    # The maximum possible first turn intensity is determined by the
    # calculation in waveform.cpp:ADC_WAVEFORM::Capture() where the raw ADC
    # value (+-2^11) is rescaled into the range 0..sqrt(2)*0.5822*2^30.  Here
    # 0.5822 is the cordic correction factor.
    logMax = log10(2**30 * 0.582217 * sqrt(2))
    sl = records.calc('FT:SL',
        CALC = '20*(LOG(A)-%g)' % logMax,
        INPA = position_S,
        EGU  = 'dB',  PREC = 0)
            
    Libera.bi('FT:TRIG',
        SCAN = 'I/O Intr',
        FLNK = create_fanout('FT:FAN', *waveforms + positions + [sl]))

    longInOut('FT:OFF', LOPR = 0, HOPR = 1023)
    longInOut('FT:LEN', LOPR = 1, HOPR = 1024)


# Booster ramp support records.  Decimated waveforms for overview of entire
# 100ms booster ramp.
def Booster():
    waveforms = \
        [Waveform('BN:WF%s' % button, 3040) for button in 'ABCDXYSQ'] + \
        [Waveform('BN:WFS%s' % button, 190) for button in 'XYS'] 

    Libera.bi('BN:TRIG',
        SCAN = 'I/O Intr',
        FLNK = create_fanout('BN:FAN', *waveforms))

    # Axes for user friendly graphs, used to label the WF and WFS waveforms.
    # Each labels the time axis in milliseconds.
    Waveform('BN:AXIS', 3040, 'FLOAT', PINI='YES')
    Waveform('BN:AXISS', 190, 'FLOAT', PINI='YES')
        


# Turn-by-turn snapshot records.  Access to long waveforms captured on
# request.  Typically used for tune measurements.  Up to 200,000 points can
# be captured in one request.
def TurnByTurn():
    # Number of points successfully captured by the last trigger.
    captured = Libera.longin('TT:CAPTURED')
    # Write a one to request a refill of the buffer.
    arm = Libera.bo('TT:ARM',
        ZNAM = 'Not armed',
        ONAM = 'Trigger enabled')
    # This indicates whether the buffer has been filled.
    ready = Libera.bi('TT:READY',
        SCAN = 'I/O Intr',
        PINI = 'YES',
        ZNAM = 'No waveform',
        ONAM = 'Waveform ready')
    # When the ready record makes a transition into the ready state re-enable
    # the arm record by setting it to zero.
    rearm = records.calcout(
        'TT:REARM',
        OUT  = arm,
        INPA = ready,
        CALC = '!A',
        OOPT = 'Transition To Zero',
        DOPT = 'Use CALC')
    ready.FLNK = create_fanout('TT:FANA', rearm, captured)

    # Note: need to update FREERUN when caputure length changes...
    freerunIn, freerunOut = boolInOut('TT:FREERUN',
        ZNAM = 'Normal', ONAM = 'Free Run')

    waveforms = [
        Waveform('TT:WF%s' % button, 1024)
        for button in
            list('ABCDXYQS') + [b + a for b in 'ABCD' for a in 'IQ']]
            
    longInOut('TT:LENGTH', LOPR = 1, HOPR = 1024)
    caplenIn, caplenOut = longInOut('TT:CAPLEN', LOPR = 1, HOPR = 200000)
    # A slightly tricksy hack: the FREERUN flag can be reset in two
    # circumstances -- when explicitly set to false, but also when the
    # capture length becomes too large.
    caplenIn.FLNK = freerunIn

    Libera.longout('TT:OFFSET_S', 'TT:OFFSET',
        OMSL = 'supervisory', LOPR = 0, HOPR = 200000)
    offset = Libera.longin('TT:OFFSET', PINI = 'YES')

    Libera.bi('TT:TRIG',
        SCAN = 'I/O Intr',
        FLNK = create_fanout('TT:FANB', *waveforms + [offset]),
        DESC = 'Trigger FT waveform capture')

    

# Configuration control records.  Used for setting button or stripline
# geometry and 
def Config():
    boolInOut('CF:DIAG', ZNAM = 'VERTICAL', ONAM = 'DIAGONAL')
    aInOut('CF:KX', 0, 32,   PREC = 4, EGU = 'mm')
    aInOut('CF:KY', 0, 32,   PREC = 4, EGU = 'mm')
    aInOut('CF:KQ', 0, 32,   PREC = 4, EGU = 'mm')
    aInOut('CF:X0', -16, 16, PREC = 4, EGU = 'mm')
    aInOut('CF:Y0', -16, 16, PREC = 4, EGU = 'mm')
    aInOut('CF:GA', 0, 1.5, VAL = 1, PREC = 4)
    aInOut('CF:GB', 0, 1.5, VAL = 1, PREC = 4)
    aInOut('CF:GC', 0, 1.5, VAL = 1, PREC = 4)
    aInOut('CF:GD', 0, 1.5, VAL = 1, PREC = 4)

    attenwf = Libera.waveform('CF:ATTWF',
        NELM = 8, FTVL = 'LONG', PINI = 'YES')
    Libera.longout('CF:ATT1_S', 'CF:ATT1',
        OMSL = 'supervisory', LOPR = 0, HOPR = 31, FLNK = attenwf)
    Libera.longout('CF:ATT2_S', 'CF:ATT2',
        OMSL = 'supervisory', LOPR = 0, HOPR = 31, FLNK = attenwf)

    longInOut('CF:SW', LOPR = 0, HOPR = 15)


def SlowAcquisition():
    sa = [Libera.ai('SA:' + name, MDEL = -1, PREC = 4, EGU = 'mm')
        for name in 'ABCDXYSQ']
    Libera.bi('SA:TRIG',
        SCAN = 'I/O Intr',
        FLNK = create_fanout('SA:FAN', *sa))



# Not yet used ...
#   These interfaces will probably change drastically.

def Fast():
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



# Finally generate and output the supported records.
    
FirstTurn()
Booster()
TurnByTurn()
SlowAcquisition()

Config()

WriteRecords(sys.argv[1])
