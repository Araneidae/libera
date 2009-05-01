# This file is part of the Libera EPICS Driver,
# Copyright (C) 2009 Michael Abbott, Diamond Light Source Ltd.
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

# Database for Libera 2.0 specific features.  This is only loaded if a 2.0
# FPGA is detected.

import sys

from common import *


def MeanSum():
    SetChannelName('MS')
    Enable()
    Trigger(False, [
        longIn('COUNT',     DESC = 'Samples in last interval'),
        longIn('MEANS',     DESC = 'Mean S in last interval'),
        longIn('DELTAS',    DESC = 'S delta'),
        aIn('MEANI', 0, 500, 1e-5, 'mA', 3,
            DESC = 'Mean current in inverval'),
        aIn('DELTAI', -500, 500, 1e-5, 'mA', 3,
            DESC = 'Current delta'),
        aIn('MEANP', -80, 10, 1e-6, 'dBm', 3,
            DESC = 'Mean input power'),
        ])
    UnsetChannelName()

    
def SpikeRemoval():
    SPIKE_DEBUG_BUFLEN = 128
    
    SetChannelName('CF')
    boolOut('SR:ENABLE', 'Disabled', 'Enabled',
        DESC = 'Enable FA spike removal')
    longOut('SR:AVEWIN',
        DESC = 'Spike average window length')
    longOut('SR:AVESTOP',
        DESC = 'Spike average window stop')
    longOut('SR:SPIKEST',
        DESC = 'Start of spike window')
    longOut('SR:SPIKEWIN',
        DESC = 'Length of spike window')

    debugwf = Waveform('SR:DEBUGWF', SPIKE_DEBUG_BUFLEN,
        DESC = 'Spike removal debug data')
    
    UnsetChannelName()
    

def PmTriggering():
    pass
    

# Finally generate and output the supported records.

MeanSum()
SpikeRemoval()
PmTriggering()


WriteRecords(sys.argv[1])
