# This file is part of the Libera EPICS Driver,
# Copyright (C) 2005-2009 Michael Abbott, Diamond Light Source Ltd.
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


# All this palaver is to pick up the IOC builder version from
# configure/RELEASE so that it can be maintained properly.
builder_version = os.environ['IOCBUILDER']
if builder_version == '':
    # Assume iocbuilder already on python path, do nothing more
    pass
elif builder_version[0] == '/':
    sys.path.append(builder_version)
else:
    from pkg_resources import require
    require('iocbuilder==%s' % builder_version)
from iocbuilder import ModuleVersion, TemplateRecordNames, ConfigureTemplate


# Ensure that we can find the Libera dbd files.  The home directory is where
# the top level Libera directory has been placed.
HomeDir = os.path.realpath(
    os.path.join(os.path.dirname(sys.argv[0]), '../..'))


class LiberaRecordNames(TemplateRecordNames):
    super = TemplateRecordNames
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

ConfigureTemplate(record_names = RecordNames)
ModuleVersion('Libera', home=HomeDir, use_name=False)

from iocbuilder import *


def ChannelName():
    return RecordNames.ChannelName


# This class wraps the creation of records which talk directly to the
# Libera device driver.
class Libera(Device):
    DbdFileList = ['libera']

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
            setattr(record, self.addr_name, '@' + ChannelName)

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

def longOut(name, DRVL=None, DRVH=None, EGU=None, **fields):
    return Libera.longout(name + '_S', address=name,
        OMSL = 'supervisory',
        DRVL = DRVL, DRVH = DRVH, **fields)


# Field name prefixes for mbbi/mbbo records.
_mbbPrefixes = [
    'ZR', 'ON', 'TW', 'TH', 'FR', 'FV', 'SX', 'SV',     # 0-7
    'EI', 'NI', 'TE', 'EL', 'TV', 'TT', 'FT', 'FF']     # 8-15

# Adds a list of (option, value [,severity]) tuples into field settings
# suitable for mbbi and mbbo records.
def process_mbb_values(fields, option_values):
    def process_value(fields, prefix, option, value, severity=None):
        fields[prefix + 'ST'] = option
        fields[prefix + 'VL'] = value
        if severity:
            fields[prefix + 'SV'] = severity
    for prefix, value in zip(_mbbPrefixes, option_values):
        process_value(fields, prefix, *value)

def mbbOut(name, *option_values, **fields):
    process_mbb_values(fields, option_values)
    return Libera.mbbo(name + '_S', address=name,
        OMSL = 'supervisory', **fields)

def mbbIn(name, *option_values, **fields):
    process_mbb_values(fields, option_values)
    return Libera.mbbi(name, **fields)


def stringIn(name, **fields):
    return Libera.stringin(name, **fields)


def Waveform(name, length, FTVL='LONG', **fields):
    return Libera.waveform(name,
        SCAN = 'Passive',
        NELM = length,
        FTVL = FTVL,
        **fields)

def WaveformOut(name, length, FTVL='LONG', SCAN='Passive', **fields):
    return Libera.waveform(name + '_S', address=name,
        SCAN = SCAN, NELM = length, FTVL = FTVL, **fields)


# An InOut record is a fairly complex construction: an output record controls
# a PV, but the controlled value may spontaneously change underfoot.  We
# handle this with three records:
#
#   output - this is the main controlling record
#   readback - this uses 'I/O Intr' scan to monitor the controlled value
#   copy - a third record monitors the readback and copies any changes back
#       into the output record
def __makeInOut(record, mkIn, mkOut):
    def Builder(name, *vargs, **fields):
        output = mkOut(name, *vargs, **fields)
        readback = mkIn(name + '_R',
            SCAN = 'I/O Intr',
            *vargs, **fields)
        readback.FLNK = record(name + '_C',
            DOL = readback, OMSL = 'closed_loop', OUT = PP(output))
    return Builder

boolInOut = __makeInOut(records.bo, boolIn, boolOut)
mbbInOut  = __makeInOut(records.mbbo, mbbIn, mbbOut)
longInOut = __makeInOut(records.longout, longIn, longOut)


__all__ = [
    'Libera', 'ChannelName',
    'aIn',      'aOut',
    'boolIn',   'boolOut',  'boolInOut',
    'longIn',   'longOut',  'longInOut',
    'mbbIn',    'mbbOut',   'mbbInOut',
    'stringIn', 'Waveform', 'WaveformOut']
