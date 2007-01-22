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

# Support code for Python epics database generation.

import sys
import os

import epics


# Ensure that we can find the Libera dbd files.  The home directory is where
# the top level Libera directory has been placed.
HomeDir = os.path.realpath(
    os.path.join(os.path.dirname(sys.argv[0]), '../../..'))
epics.LibVersion('Libera', home=HomeDir)


class LiberaRecordNames(epics.TemplateRecordNames):
    super = epics.TemplateRecordNames
    __all__ = super.__all__ + ['SetChannelName', 'UnsetChannelName']

    def __init__(self):
        self.super.__init__(self)
        self.ChannelName = None

    def SetChannelName(self, name):
        assert self.ChannelName == None, 'Channel name already set'
        self.ChannelName = name

    def UnsetChannelName(self):
        self.ChannelName = None

    def RecordName(self, name):
        return self.super.RecordName(self, self.GetChannelName(name))

    def GetChannelName(self, name):
        if self.ChannelName:
            return '%s:%s' % (self.ChannelName, name)
        else:
            return name


RecordNames = LiberaRecordNames()
epics.Configure(recordnames = RecordNames)
from epics import *


def ChannelName():
    return RecordNames.ChannelName


# This class wraps the creation of records which talk directly to the
# Libera device driver.
class Libera(hardware.Device):
    @classmethod
    def LoadLibrary(cls):
        cls.LoadDbdFile('libera.dbd')

    class makeRecord:
        def __init__(self, builder, addr_name):
            self.builder = getattr(records, builder)
            self.addr_name = addr_name

        def __call__(self, name, address=None, **fields):
            if address is None:
                address = name
            record = self.builder(name, **fields)
            record.DTYP = 'Libera'
            ChannelName = RecordNames.GetChannelName(address)
            setattr(record, self.addr_name, ChannelName)

            # Check for a description, make a report if none given.
            if 'DESC' not in fields:
                print 'No description for', ChannelName
                
            return record

    @classmethod
    def init(cls):
        for name, addr in [
                ('longin',    'INP'), ('longout',   'OUT'),
                ('ai',        'INP'), ('ao',        'OUT'),
                ('bi',        'INP'), ('bo',        'OUT'),
                ('stringin',  'INP'), ('stringout', 'OUT'),
                ('mbbi',      'INP'), ('mbbo',      'OUT'),
                ('waveform',  'INP')]:
            setattr(cls, name, cls.makeRecord(name, addr))

Libera.init()
Libera()


# ----------------------------------------------------------------------------
#           Record Generation Support
# ----------------------------------------------------------------------------


def extend(dictionary, key, value):
    assert key not in dictionary, 'key %s already in dictionary' % key
    dictionary[key] = value


# Functions for creating libera records

def aIn(name, LOPR, HOPR,
        ESLO=1e-6, EGU='', PREC=0, EOFF=0, MDEL=-1, **fields):
    return Libera.ai(name,
        MDEL = MDEL,
        ESLO = ESLO,  EOFF = EOFF,  LINR = 'LINEAR', 
        LOPR = LOPR,  HOPR = HOPR,  EGUL = LOPR,  EGUF = HOPR,  
        EGU = EGU,  PREC = PREC,
        **fields)

def aOut(name, DRVL, DRVH, ESLO=1e-6, EOFF=0, PREC=4, **fields):
    return Libera.ao(name + '_S', address=name,
        OMSL = 'supervisory', 
        ESLO = ESLO,  EOFF = EOFF,  LINR = 'LINEAR',
        PREC = PREC,  
        DRVL = DRVL,  DRVH = DRVH,
        EGUL = DRVL,  EGUF = DRVH,
        **fields)


def boolIn(name, ZNAM=None, ONAM=None, **fields):
    return Libera.bi(name, ZNAM = ZNAM, ONAM = ONAM, **fields)

def boolOut(name, ZNAM=None, ONAM=None, **fields):
    return Libera.bo(name + '_S', address=name,
        OMSL = 'supervisory', 
        ZNAM = ZNAM, ONAM = ONAM, **fields)


def longIn(name, LOPR=None, HOPR=None, EGU=None, MDEL=-1, **fields):
    return Libera.longin(name,
        MDEL = MDEL,  EGU  = EGU,
        LOPR = LOPR,  HOPR = HOPR, **fields)

def longOut(name, LOPR=None, HOPR=None, **fields):
    return Libera.longout(name + '_S', address=name,
        OMSL = 'supervisory',
        LOPR = LOPR, HOPR = HOPR, **fields)

def mbbOut(name, *option_values, **fields):
    mbbPrefixes = [
        'ZR', 'ON', 'TW', 'TH', 'FR', 'FV', 'SX', 'SV',     # 0-7
        'EI', 'NI', 'TE', 'EL', 'TV', 'TT', 'FT', 'FF']     # 8-15
    for prefix, (option, value) in zip(mbbPrefixes, option_values):
        extend(fields, prefix + 'ST', option)
        extend(fields, prefix + 'VL', value)
    return Libera.mbbo(name + '_S', address=name,
        OMSL = 'supervisory', **fields)
    

def Waveform(name, length, FTVL='LONG', **fields):
    return Libera.waveform(
        name, name,
        SCAN = 'Passive',
        NELM = length,
        FTVL = FTVL,
        **fields)




__all__ = [
    'Libera', 'ChannelName',
    'aIn',      'aOut',     'boolIn',   'boolOut',
    'longIn',   'longOut',  'mbbOut',   'Waveform']
