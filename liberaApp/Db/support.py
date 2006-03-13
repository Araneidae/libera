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
        self.__ChannelName = None

    def SetChannelName(self, name):
        assert self.__ChannelName == None, 'Channel name already set'
        self.__ChannelName = name

    def UnsetChannelName(self):
        self.__ChannelName = None

    def RecordName(self, name):
        return self.super.RecordName(self, self.GetChannelName(name))

    def GetChannelName(self, name):
        if self.__ChannelName:
            return '%s:%s' % (self.__ChannelName, name)
        else:
            return name


RecordNames = LiberaRecordNames()
epics.Configure(recordnames = RecordNames)
from epics import *


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
                setattr(record, self.addr_name,
                    RecordNames.GetChannelName(address))
            return record

    @classmethod
    def init(cls):
        for name, addr in [
                ('longin',    'INP'), ('longout',   'OUT'),
                ('ai',        'INP'), ('ao',        'OUT'),
                ('bi',        'INP'), ('bo',        'OUT'),
                ('stringin',  'INP'), ('stringout', 'OUT'),
                ('waveform',  'INP')]:
            setattr(cls, name, cls.makeRecord(name, addr))

Libera.init()
Libera()

