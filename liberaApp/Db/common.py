# This file is part of the Libera EPICS Driver,
# Copyright (C) 2009-2011 Michael Abbott, Diamond Light Source Ltd.
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

# Common definitions for libera database building.

# It is important to import support before importing iocbuilder, as the
# support module initialises iocbuilder (and determines which symbols it
# exports!)
import support
from support import *
import iocbuilder
from iocbuilder import *


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

MAX_ADC = 2**15


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

def ABCD_N():
    return [
        aIn(button + 'N', 0, 10, 1e-7, PREC = 6,
            DESC = 'Normalised %s button intensity')
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

def XYQS_(prec, suffix=''):
    channel = ChannelName()
    return [
        aIn(position + suffix, -MAX_mm, MAX_mm, 1e-6, 'mm', prec,
            DESC = '%s %s position' % (channel, position))
        for position in 'XY'] + [
        aIn('Q' + suffix, -1, 1, 1e-8, '', prec,
            DESC = '%s relative skew' % channel),
        longIn('S' + suffix, MDEL = -1,
            DESC = '%s total button intensity' % channel)]



def Enable(**fields):
    return boolOut('ENABLE', 'Disabled', 'Enabled',
        DESC = 'Enable %s mode' % ChannelName(), **fields)

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
    positions.append(
        Libera.longout(DONE, MDEL = -1, DESC = 'Report trigger done'))
    trigger = Libera.bi(TRIG, DESC = 'Trigger processing',
        SCAN = 'I/O Intr',
        TSE  = -2,          # Ensures that device timestamp is used
        FLNK = create_fanout(TRIG + 'FAN', *positions))
    for record in positions:
        record.TSEL = trigger.TIME


__all__ = support.__all__ + iocbuilder.__all__ + [
    'MAX_INT', 'MAX_mm', 'MAX_nm', 'MAX_S',
    'KB', 'MB',
    'MAX_ADC', 'RAW_ADC',
    'IQ_wf', 'ABCD_wf', 'ABCD_', 'ABCD_N', 'XYQS_wf', 'XYQS_',
    'Enable', 'Trigger' ]
