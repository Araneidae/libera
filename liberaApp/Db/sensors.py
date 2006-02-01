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

# Python script for building temperature and fan sensor database.

import sys
import os

import epics
epics.Configure(recordnames=epics.TemplateRecordNames())
 
from epics import *

HomeDir = os.path.realpath(
    os.path.join(os.path.dirname(sys.argv[0]), '../../..'))
LibVersion('ReadFile', 'Libera', home=HomeDir)


class ReadFile(hardware.Device):
    @classmethod
    def LoadLibrary(cls):
        cls.LoadDbdFile('dbd/device.dbd')
    



def TemperatureSensor(name, file, description):
    records.longin(name,
        DTYP = 'ReadFile',
        INP  = '/proc/sys/dev/sensors/max1617a-i2c-0-%s|2' % file,
        SCAN = DefaultScanRate,
        EGU  = 'deg C',
        LOPR = 20,
        HOPR = 60,
        DESC = description)

def FanSensor(name, file, description):
    records.longin(name,
        DTYP = 'ReadFile',
        INP  = '/proc/sys/dev/sensors/max6650-i2c-0-%s' % file,
        SCAN = DefaultScanRate,
        EGU  = 'RPM',
        LOPR = 0,
        HOPR = 6000,
        DESC = description)

def FanControl(name, file, description):
    pass

def MemInfo(name, field, description):
    records.longin(
        name,
        DTYP = 'ReadFile',
        INP  = '/proc/meminfo|%d,1' % field,
        SCAN = DefaultScanRate,
        EGU  = 'Bytes',
        LOPR = 0,
        HOPR = 64 * 1024 * 1024,   # 64 MB
        DESC = description)
        

DefaultScanRate = '10 second'


ReadFile()

TemperatureSensor('TEMP1', '29/temp1', 'Main PCB Temperature')
TemperatureSensor('TEMP2', '29/temp2', '(unused)')
 
FanSensor( 'FAN1', '48/fan1',  'Back fan speed')
FanControl('FAN1', '48/speed', 'Back fan set speed')
FanSensor( 'FAN2', '4b/fan1',  'Front fan speed')
FanControl('FAN2', '4b/speed', 'Front fan set speed')
 
MemInfo('USED',  2, 'Total used memory')
MemInfo('FREE',  3, 'Total memory free')
MemInfo('CACHE', 6, 'Total cached memory')

WriteRecords(sys.argv[1])
