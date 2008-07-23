# This file is part of the Libera EPICS Driver,
# Copyright (C) 2008  Michael Abbott, Diamond Light Source Ltd.
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

# Python script to compute low pass filter.

# Note that the filter really ought to be loaded at run time rather than
# being compiled in.

from __future__ import division

from numpy import *
import math


print '/* Filter definition for raw ADC filtering.'
print ' * Automatically generated by filter-header.py. */'


def ConvolveSequence(*s):
    result = array(s[0], dtype = float)
    for a in s[1:]:
        result = convolve(result, array(a, dtype = float))
    return result

def FilterFromAngles(*angles):
    '''Computes a (low pass) filter with zeros on the unit circle in the
    z-plane at the specified angles from -pi: the angles are specified in
    multiples of pi.  A zero at -1 is also added.'''
    return ConvolveSequence([1, 1],
        *[[1, 2 * cos(theta), 1]
            for theta in pi * array(angles)])


# Low pass filter
#
# This low pass filter is designed to be applied to the raw ADC waveform
# after it has been frequency shifted so that the intermediate frequency is
# close to DC.  As our intermediate frequency is close to 0.25 f_s (pi/2)
# it's good enough to frequency shift by pi/2, and this has some extra
# advantages.
#   After frequency shifting the spectrum has the following features (working
# around the unit circle in multiples of sample frequency):
#
#   0 (DC)  The signal of interest.
#   0.25    A small DC residual
#   0.5     The mirror image of the signal, to be filtered out
#   0.75    The second harmonic of the signal
#
# We deal with this by synthesising a filter with zeros at +-0.25 and 0.5,
# together with two further pairs of zeros close by:
#
#   0        0.25      0.5      -0.25       0
#   +----|----+----|----+----|----+----|----+
#   ###                                   ###   Signal of interest
#             #                  ###            DC residual, second harmonic
#                     #####                     Alias at opposite frequency
#             0 0     0 0 0     0 0             7 zeros of filter
#
# The filter needs to be exactly 8 points long: the code in firstTurn.cpp is
# crafted on this assumption, so we synthesise using 7 zeros as above.

# Angles specified here in units of (1-angle)f_s/2 or pi(1-angle). 
# filter = FilterFromAngles(1/6, 1/3, 1/2)
filter = FilterFromAngles(0.05, 0.45, 0.5)

# Scale the filter so that its values are in the range 0..2**16 (the filter
# is entirely positive).  Given signed 16-bit input values the product at
# each point fits in a signed 32-bit integer.
filter = int_(filter / max(abs(filter)) * 2**16)

print
print 'static const int %s[%d] =' % ('FilterADC', len(filter))
print '{'
for x in filter:
    print '    %d,' % x
print '};'
print

filter_sum = sum(filter)
print '/* Filter sum = %d = 2**%g. */' % (filter_sum, math.log(filter_sum, 2))