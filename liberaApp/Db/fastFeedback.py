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

# Python script for building database for fast feedback control records

import sys
from math import *

from support import *
from iocbuilder import *


# 10 bit BPM identifier, so range is 0..1023
MAX_ID = 2**10 - 1

def FastFeedback():
    SetChannelName('FF')

    longIn('VERSION', PINI = 'YES', DESC = 'FPGA firmware version')

    time_raw = longIn('PROCESS_TIME', DESC = 'Total communication time')
    time_us = records.calc('PROCESS_TIME_US',
        CALC = 'A/B',       EGU = 'us',     PREC = 2,
        INPA = time_raw,    INPB = 106.25,
        DESC = 'Communication time in us')
    inputs = [
        longIn('TIMEFRAME',    DESC = 'Timeframe counter'),
        time_raw, time_us,
        longIn('BPM_COUNT',    DESC = 'Number of transmitters seen'),
        longIn('SOFT_ERR',     DESC = 'Total soft error count'),
        longIn('FRAME_ERR',    DESC = 'Total frame error count'),
        longIn('HARD_ERR',     DESC = 'Total hard error count'),
        longIn('RXFIFO',       DESC = 'Max RX FIFO length'),
        longIn('TXFIFO',       DESC = 'Max TX FIFO length'),
        Waveform('TOA_MIN', 256,   DESC = 'Min Time of Arrival'),
        Waveform('TOA_MAX', 256,   DESC = 'Max Time of Arrival'),
        Waveform('RCB', 256,   DESC = 'Receive Count'),
    ] + [
        field
        for i in (1, 2, 3, 4)
        for field in [
            longIn('LINK%d:PARTNER' % i, 0, MAX_ID,
                DESC = 'Rocket IO channel partner'),
            longIn('LINK%d:SOFT_ERR' % i,  DESC = 'Soft error count'),
            longIn('LINK%d:FRAME_ERR' % i, DESC = 'Frame error count'),
            longIn('LINK%d:HARD_ERR' % i,  DESC = 'Hard error count'),
            longIn('LINK%d:RX_CNT' % i,    DESC = 'Received packet count'),
            longIn('LINK%d:TX_CNT' % i,    DESC = 'Transmitted packet count'),
            boolIn('LINK%d:TX_UP' % i, 'Link Down', 'Link Up',
                DESC = 'Transmit link status'),
            boolIn('LINK%d:RX_UP' % i, 'Link Down', 'Link Up',
                DESC = 'Receive link status'),
            longIn('LINK%d:RXFIFO' % i,    DESC = 'RX FIFO length'),
            longIn('LINK%d:TXFIFO' % i,    DESC = 'TX FIFO length'),
        ]]

    boolOut('PROCESS',
        DESC = 'Update FF fields',
        SCAN = '1 second', FLNK = create_fanout('FANOUT', *inputs))

    for i in range(4):
        i = i+1
        boolOut('LINK%d:ENABLE' % i, 'Disabled', 'Enabled',
            DESC = 'Link (Rocket IO) enable')
        mbbOut('LINK%d:LOOPBACK' % i,
            ('No loopback', 0), ('Serial', 1), ('Parallel', 2),
            DESC = 'Rocket IO loopback')

    longOut('BPMID', 0, MAX_ID, DESC = 'BPM id')
    longOut('FRAMELEN', DESC = 'Timeframe length')

    # CR 1 - position or time
    boolOut('DATA_SELECT', 'Positions', 'Timestamps',
        DESC = 'Position data select')
    # X and Y payload selection
    payload_options = [(opt, val + 8) for val, opt in enumerate('ABCDSQXY')]
    mbbOut('XPAYLOAD', *payload_options,
        DESC = 'Payload selection for FA X')
    mbbOut('YPAYLOAD', *payload_options,
        DESC = 'Payload selection for FA Y')

    boolOut('STOP',  'Stop',  DESC = 'Stop fast feedback')
    boolOut('START', 'Start', DESC = 'Start fast feedback')
    boolOut('RESET_ERR', 'Reset Errors', DESC = 'Reset error counts')

    UnsetChannelName()


FastFeedback()

WriteRecords(sys.argv[1])
