#!/usr/bin/env python

'''Test implementation of DSC investigation using pylab and mlab.'''

from __future__ import division

import sys, os
import pprint

# There is a slight cock-up in the system installation: version 0.84 is
# hard-wired into the global installation, which means that we have to remove
# it from the path if we want require() to find the latest version.  We only
# do this as a top level module, as this is caller's responsibility.  Note
# that this needs to be done before importing pkg_resources!
if __name__ == '__main__':
    sys.path = [s for s in sys.path if s.find('/matplotlib-') == -1]
from pkg_resources import require

require('matplotlib')
require('dls.ca2')

os.environ['EPICS_CA_MAX_ARRAY_BYTES'] = '1000000'
from dls.ca2 import catools
from pylab import *
from MLab import *
#from matplotlib.mlab import *

import time


def unwrap(array):
    '''Implements array unwrapping.  Has to be along the last axis due to
    problems in expressing a suitable arbitrary slice.'''
    deltas = diff(array, axis = -1)
    offsets = cumsum(deltas < -pi, -1) - cumsum(deltas > pi, -1)
    return concatenate(
        (array[..., 0:1], array[..., 1:] + 2 * pi * offsets), -1)

def big_zeros(*dims):
    '''Returns zeros in an unlimited number of dimensions.  For some bizzare
    reason the built-in zeros is limited to three!'''
    return reshape(zeros(prod(dims), Complex), dims)

def plot_array(data):
    for f in data:
        plot(f)

def plot_complex(data, plot_circle=False, blob='x'):
    if len(shape(data)) > 1:
        for f in data:
            plot(f.real, f.imag, blob)
    else:
        plot(data.real, data.imag, blob)

    r = abs(data)
    while len(shape(r)):
        r = mean(r)
    if plot_circle:
        circle = r*exp(1j*linspace(0, 2*pi, 1000))
        plot(circle.real, circle.imag, ':')
        

def std(m,axis=0):
    """std(m,axis=0) returns the standard deviation along the given
    dimension of m.  The result is unbiased with division by N-1.
    If m is of integer type returns a floating point answer.
    """
    x = asarray(m)
    n = float(x.shape[axis])
    mx = asarray(mean(x,axis))
    if axis < 0:
        axis = len(x.shape) + axis
    mx.shape = mx.shape[:axis] + (1,) + mx.shape[axis:]
    x = abs(x - mx)
    return sqrt(add.reduce(xx, axis)/(n-1.0))


def caGet(ioc, pv, datatype = None):
    return catools.caget('%s:%s' % (ioc, pv), datatype = datatype).value

def caPut(ioc, pv, value, datatype = None):
    catools.caput('%s:%s' % (ioc, pv), value, datatype = datatype)


def GetIq(ioc, length):
    # Interrogate the default window length and set up trigger capture and
    # readout windows.
    window_length = caGet(ioc, 'TT:WFAI.NELM', catools.dbr_long)
    assert length <= window_length, 'Don\'t support long waveforms yet'
    caPut(ioc, 'TT:CAPLEN_S', length)
    caPut(ioc, 'TT:LENGTH_S', window_length)

    # Perform the trigger capture handshake.
    caPut(ioc, 'TT:READY', 0)
    while caGet(ioc, 'TT:READY') != 0:
        time.sleep(0.01)
    caPut(ioc, 'TT:ARM', 1)
    while caGet(ioc, 'TT:READY') == 0:
        time.sleep(0.01)

    captured = caGet(ioc, 'TT:CAPTURED')
    raw_iq = array([wf.dbr.value for wf in catools.caget(
        ['%s:TT:WF%s%s' % (ioc, button, iq)
            for button in 'ABCD' for iq in 'IQ'])])
    raw_iq = reshape(raw_iq[:, 0:length], (4, 2, length))
    return raw_iq[:, 0, :] + 1j * raw_iq[:, 1, :]


def geo_mean(data, axis=0):
    return product(data**(1./shape(data)[axis]), axis)
    

def ReadRawData(ioc, sw_count):
#     # First ensure the DSC is operating with unit gains and the switches are
#     # rotating.
#     caPut(ioc, 'CF:DSC_S',    'Unity gains', catools.dbr_string)
#     caPut(ioc, 'CF:AUTOSW_S', 'Automatic',   catools.dbr_string)

    # Now read a block of data.
    count = 2048
    iq = GetIq(ioc, count)

    # Find the switch marker positions.  These are encoded into the least
    # significant bit of the WFAI data
    sw_len = 40
    cycle_len = sw_count * sw_len
    markers = find(diff(iq[1, 1:-cycle_len].real % 2) > 0)
    nm = len(markers)

    # To help with subsequent process, normalise the data stream.
    iq /= mean(mean(iq))

    # Now reshape and chop iq to bring all the waveforms for the same switch
    # position together.  From each marker we extract 320 points, chopped
    # into 40 points per switch position.
    #    The array riq is indexed thus:
    #       riq(i, n, m, t)
    #           i = button index (0..3)
    #           n = switch index (0..7)
    #           m = marker index
    #           t = time into individual waveform (0..29)
    # The first 10 points after each switch position are discarded to allow
    # time for the switching transient to settle.
    nsw = 30
    riq = big_zeros(4, sw_count, nm, nsw)
    for i, n in enumerate(markers):
        riq[:, :, i, :] = reshape(
            iq[:, n:n+cycle_len], (4, sw_count, 40))[:, :, 10:40]
    # Finally cast away a dimension by running all the marker data together.
    riq = reshape(riq, (4, sw_count, nm*nsw))

    # Compute the mean and standard deviations of the condensed data
    m = mean(riq, -1)
    s = std(riq, -1)

    return iq, riq, m, s


# Permutation array mapping buttons to channels: button[i] is processed by
# channel[perms[n][i]] in switch position n.
perms = [
    [3, 2, 1, 0],  [3, 1, 2, 0],  [0, 2, 1, 3],  [0, 1, 2, 3],
    [3, 2, 0, 1],  [3, 1, 0, 2],  [0, 2, 3, 1],  [0, 1, 3, 2],
    [2, 3, 1, 0],  [1, 3, 2, 0],  [2, 0, 1, 3],  [1, 0, 2, 3],
    [2, 3, 0, 1],  [1, 3, 0, 2],  [2, 0, 3, 1],  [1, 0, 3, 2]
]


# Records the cycle of switch positions.
switches = [3, 7, 15, 11, 0, 4, 12, 8]
# Brilliance only needs 4 switches (?) */
switches16 = [15, 0, 9, 6]


def DoTest(ioc, switches=switches):
    iq, riq, ms, ss = ReadRawData(ioc, len(switches))

    subplot(3, 2, 1)
    plot_array(unwrap(angle(iq)))
    title('Raw data: phase')

    subplot(3, 2, 2)
    plot_array(abs(iq))
    title('Raw data: magnitude')

    for i, m in enumerate(riq):
        subplot(3, 4, i+5)
#         plot_array(unwrap(angle(m)))
#         subplot(6, 4, i+13)
        plot_array(abs(m))

    subplot(3, 3, 7)
    plot_complex(ms)
    axis('equal')
    
    subplot(3, 3, 8)
    plot_complex(geo_mean(ms, 1))
    m = mean(ms, 1)
    plot(m.real, m.imag, 'o')


#     pprint.pprint(ms)
#     pprint.pprint(ss)
#     pprint.pprint(transpose(abs(ms)))
#     pprint.pprint(transpose(angle(ms)))


def DoTest2(ioc):
    iq, riq, ms, ss = ReadRawData(ioc, 8)
    plot_complex(ms)
    axis('equal')



sw_len = 40     # Number of samples in a single switch position
sw_count = 8    # Number of switches in a complete switch cycle
cycle_len = sw_count * sw_len   # Samples in complete switch cycle


def GetIqSC(ioc):
    raw_iq = array([wf.dbr.value for wf in catools.caget(
        ['%s:SC:WF%s%s' % (ioc, button, iq)
            for button in 'ABCD' for iq in 'IQ'])])
    raw_iq = reshape(raw_iq, (4, 2, 2048))
    return raw_iq[:, 0, :] + 1j * raw_iq[:, 1, :], raw_iq[0, 0, :] & 1

def GetDigest(ioc):
    raw_digest = array(catools.caget('%s:SC:IQDIGEST' % ioc).dbr.value)
    n = len(raw_digest)
    return reshape(
        [x + 1j*y for x, y in reshape(raw_digest, (n/2, 2))], (8, 4))


def DigestWaveform(iq, markers, truncate=False):
    sw_offset = 10  # Number of samples to skip over at start of cycle
    
    # Find the switch marker positions.  These are encoded into the least
    # significant bit of the WFAI data
    markers = find(diff(markers[:-cycle_len]) > 0) + 1
    nm = len(markers)

    # Hack to IQ data to emulate truncation in true SC implementation.
    if truncate:
        iq = copy.copy(iq)
        for i in range(4):
            for j in range(2048):
                x = iq[i,j]
                iq[i,j] = (int(x.real) >> 8) + 1j * (int(x.imag) >> 8)
    
    # Now reshape and chop iq to bring all the waveforms for the same switch
    # position together.  From each marker we extract 320 points, chopped
    # into 40 points per switch position.
    #    The array riq is indexed thus:
    #       riq(i, n, m, t)
    #           i = button index (0..3)
    #           n = switch index (0..7)
    #           m = marker index
    #           t = time into individual waveform (0..29)
    # The first 10 points after each switch position are discarded to allow
    # time for the switching transient to settle.
    nsw = sw_len - sw_offset
    riq = big_zeros(4, sw_count, nm, nsw)
    nn = size(riq)
    for i, n in enumerate(markers):
        riq[:, :, i, :] = reshape(iq[:, n:n+cycle_len],
            (4, sw_count, sw_len))[:, :, sw_offset:sw_len]
    # Finally cast away a dimension by running all the marker data together.
    riq = reshape(riq, (4, sw_count, nm*nsw))

    # Compute some basic statistics about the reshaped data: mean of the
    # data, variance of the data and the minimum absolute value.
    m = mean(riq, -1)
    rm = reshape(repeat(m, nm*nsw, -1), shape(riq))
    v = sum(sum(sum(abs(riq-rm)**2)))
    s = sqrt(v / nn) / min(min(abs(m)))

    if truncate:
        m = 256 * m
    return transpose(m), s
    

def DoTestSC(ioc):
    '''Reads the SC data and tries to replicate the calculation.'''

    iq, markers = GetIqSC(ioc)
    digest = GetDigest(ioc)
    m, s = DigestWaveform(iq, markers, True)

    buttons = mean(m, axis=0)
    offsets = m - repeat(reshape(buttons, (1,4)), 8)

    print 'Deviation =', 100 * s

    subplot(2,2,1)
    plot_array(abs(iq))

    subplot(2,2,2)
    plot_array(180./pi*angle(iq))

    subplot(2,2,3)
    plot_complex(m)
    plot_complex(buttons, blob='o')

    subplot(2,2,4)
    plot_complex(offsets)
    show()

    
if __name__ == '__main__':
    if len(sys.argv) not in [2, 3]:
        print 'Usage: %s <ioc> [<png-file>]' % sys.argv[0]
        sys.exit(1)

    DoTestSC(sys.argv[1])

#     if len(sys.argv) > 2:
#         savefig(sys.argv[2] + '.png')
# 
#     show()

