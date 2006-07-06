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

# Support and epics must be imported in this order.
import support 
from epics import *


class ReadFile(hardware.Device):
    LibraryName = 'Libera'
    @classmethod
    def LoadLibrary(cls):
        cls.LoadDbdFile('device.dbd')
    


DefaultScanRate = '10 second'


def TemperatureSensor(name, file, description):
    return records.longin(name,
        DTYP = 'ReadFile',
        INP  = '/proc/sys/dev/sensors/max1617a-i2c-0-%s|2' % file,
        EGU  = 'deg C',
        LOPR = 20,   HOPR = 60,
        HIGH = 45,   HSV  = 'MINOR',
        HIHI = 50,   HHSV = 'MAJOR',
        LOLO = 0,    LLSV = 'MINOR',
        DESC = description)

def FanSensor(name, file, description):
    return records.longin(name,
        DTYP = 'ReadFile',
        INP  = '/proc/sys/dev/sensors/max6650-i2c-0-%s' % file,
        EGU  = 'RPM',
        LOPR = 0,    HOPR = 6000,
        LOW  = 4000, LSV  = 'MINOR',
        LOLO = 1000, LLSV = 'MAJOR',
        DESC = description)

MB = 1024 * 1024
def MemInfo(name, field, description, SCAN=DefaultScanRate):
    return records.longin(
        name,
        DTYP = 'ReadFile',
        INP  = '/proc/meminfo|%d,1' % field,
        SCAN = SCAN,
        EGU  = 'Bytes',
        LOPR = 0,
        HOPR = 64 * MB,
        LOW  = 32 * MB,   LSV  = 'MINOR',
        LOLO = 16 * MB,   LLSV = 'MAJOR',
        DESC = description)
        


ReadFile()

temp1 = TemperatureSensor('TEMP1', '29/temp1', 'Main PCB Temperature')
fan1 = FanSensor('FAN1', '48/fan1', 'Back fan speed')
fan2 = FanSensor('FAN2', '4b/fan1', 'Front fan speed')
 
#MemInfo('USED',  2, 'Total used memory')
#MemInfo('CACHE', 6, 'Total cached memory')
free = MemInfo('FREE',  3, 'Total memory free', SCAN='Passive')

records.calc('HEALTH',
    SCAN = DefaultScanRate,
    CALC = '1',
    INPA = PP(MS(temp1)),
    INPB = PP(MS(fan1)),
    INPC = PP(MS(fan2)),
    INPD = PP(MS(free)))

WriteRecords(sys.argv[1])
