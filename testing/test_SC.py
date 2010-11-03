#!/usr/bin/env python2.4

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
require('dls.green')

os.environ['EPICS_CA_MAX_ARRAY_BYTES'] = '1000000'
from dls.ca2 import catools
from dls.green import start_gui, asleep
#from pylab import *
import pylab
#from MLab import *
from numpy import *
#from matplotlib.mlab import *

import time


def unwrap(array):
    '''Implements array unwrapping.  Has to be along the last axis due to
    problems in expressing a suitable arbitrary slice.'''
    deltas = diff(array, axis = -1)
    offsets = cumsum(deltas < -pi, -1) - cumsum(deltas > pi, -1)
    return concatenate(
        (array[..., 0:1], array[..., 1:] + 2 * pi * offsets), -1)

def std(m,axis=0):
    '''std(m,axis=0) returns the standard deviation along the given
    dimension of m.  The result is unbiased with division by N-1.
    If m is of integer type returns a floating point answer.
    '''
    x = asarray(m)
    n = float(x.shape[axis])
    mx = asarray(mean(x,axis))
    if axis < 0:
        axis = len(x.shape) + axis
    mx.shape = mx.shape[:axis] + (1,) + mx.shape[axis:]
    xx = abs(x - mx)
    return sqrt(add.reduce(xx, axis)/(n-1.0))


def geo_mean(a, axis):
    '''Geometric mean along the given axis.'''
    if axis is None:
        n = size(a)
    else:
        n = a.shape[axis]
    return product(a, axis)**(1/n)


def cis(r, a):
    return r * exp(1j * a)


def plot_array(data):
    for f in data:
        pylab.plot(f)

def plot_complex(data, plot_circle=False, blob='x'):
    if len(shape(data)) > 1:
        for f in data:
            pylab.plot(f.real, f.imag, blob)
    else:
        pylab.plot(data.real, data.imag, blob)

    r = abs(data)
    while len(shape(r)):
        r = mean(r)
    if plot_circle:
        circle = r*exp(1j*linspace(0, 2*pi, 1000))
        pylab.plot(circle.real, circle.imag, ':')



def find(v):
    return v.nonzero()[0]


def caGet(ioc, pv, datatype = None):
    return catools.caget('%s:%s' % (ioc, pv),
        datatype = datatype, timeout = 5).value

def caPut(ioc, pv, value, datatype = None):
    catools.caput('%s:%s' % (ioc, pv), value, datatype = datatype, timeout = 5)

def caGetArray(ioc, pvs, datatype = None):
    return array([wf.dbr.value for wf in catools.caget(
        ['%s:%s' % (ioc, pv) for pv in pvs])], timeout = 5)


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
    wfs = catools.caget(
        ['%s:TT:WF%s%s' % (ioc, button, iq)
            for button in 'ABCD' for iq in 'IQ'],
        timeout = 5, datatype = catools.dbr_time_long)
    times = [wf.dbr.stamp for wf in wfs]

    for t in times[1:]:  assert t == times[0]

    raw_iq = array([wf.dbr.value for wf in wfs])
    raw_iq = reshape(raw_iq[:, 0:length], (4, 2, length))
    return raw_iq[:, 0, :] + 1j * raw_iq[:, 1, :], raw_iq[0, 0, :] & 1


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
#    riq = big_zeros(4, sw_count, nm, nsw)
    riq = empty((4, sw_count, nm, nsw), complex)
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
perms = array([
    [3, 2, 1, 0],  [3, 1, 2, 0],  [0, 2, 1, 3],  [0, 1, 2, 3],
    [3, 2, 0, 1],  [3, 1, 0, 2],  [0, 2, 3, 1],  [0, 1, 3, 2],
    [2, 3, 1, 0],  [1, 3, 2, 0],  [2, 0, 1, 3],  [1, 0, 2, 3],
    [2, 3, 0, 1],  [1, 3, 0, 2],  [2, 0, 3, 1],  [1, 0, 3, 2]
])


# Records the cycle of switch positions.
switches = array([3, 7, 15, 11, 0, 4, 12, 8])
# Brilliance only needs 4 switches (?) */
switches16 = array([15, 0, 9, 6])


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
    '''Returns the IQ waveform read by the current SC round.'''
    raw_iq = array([wf.dbr.value for wf in catools.caget(
        ['%s:SC:WF%s%s' % (ioc, button, iq)
            for button in 'ABCD' for iq in 'IQ'])], timeout = 5)
    n = shape(raw_iq)[1]
    raw_iq = reshape(raw_iq, (4, 2, n))
    return raw_iq[:, 0, :] + 1j * raw_iq[:, 1, :], raw_iq[0, 0, :] & 1

def GetDigest(ioc):
    '''Returns the IQ digest computed for the current SC round.'''
    raw_digest = array(
        catools.caget('%s:SC:IQDIGEST' % ioc).dbr.value)
    deviation = catools.caget('%s:SC:DEV' %ioc).dbr.value
    n = len(raw_digest)
    digest = reshape(
        [x + 1j*y for x, y in reshape(raw_digest, (n/2, 2))], (8, 4))
    return digest, deviation

PHASE_UNITY = 0x10000
def GetCompensation(ioc, f_if=2*pi*(936 % 220)/220):
    '''Returns the channel compensation terms currently in use.'''
    raw = reshape(caGetArray(ioc, ['SC:C%dRAW%d' % (channel+1, stage)
            for channel in range(4)
            for stage in (0, 1)]), (4, 2))
    return (raw[:, 0] + exp(-1j*f_if) * raw[:, 1]) / PHASE_UNITY

def GetOldCompensation(ioc, f_if=2*pi*(936 % 220)/220):
    '''Returns the channel compensation terms used for the current digest.'''
    raw = reshape(caGetArray(ioc, ['SC:LASTCK']), (4, 2))
    return (raw[:, 0] + exp(-1j*f_if) * raw[:, 1]) / PHASE_UNITY


def Truncate(iq):
    iq = copy(iq)
    for i in range(4):
        for j in range(2048):
            x = iq[i,j]
            iq[i,j] = (int(x.real) >> 8) + 1j * (int(x.imag) >> 8)
    return iq


def DigestWaveform(iq, markers, truncate=False, sw_offset=6):
    '''Emulates the computation of the IQ digest as done by the SC process.'''
    sw_offset = 10  # Number of samples to skip over at start of cycle

    # Find the switch marker positions.  These are encoded into the least
    # significant bit of the WFAI data
    markers = find(diff(markers[:-cycle_len]) > 0) + 1
    nm = len(markers)

    # Hack to IQ data to emulate truncation in true SC implementation.
    if truncate:
        iq = Truncate(iq)

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
    riq = empty((4, sw_count, nm, nsw), complex)
    nn = size(riq)
    for i, n in enumerate(markers):
        riq[:, :, i, :] = reshape(iq[:, n:n+cycle_len],
            (4, sw_count, sw_len))[:, :, sw_offset:sw_len]
    # Finally cast away a dimension by running all the marker data together.
    riq = reshape(riq, (4, sw_count, -1))

    # Compute some basic statistics about the reshaped data: mean of the
    # data, variance of the data and the minimum absolute value.
    m = mean(riq, -1)
    rm = reshape(repeat(m, nm*nsw, -1), shape(riq))
    v = sum(sum(sum(abs(riq-rm)**2)))
    s = sqrt(v / nn) / amin(ravel(abs(m)))

    return transpose(m), s


def NewDigestWaveform(iq, markers, truncate=False, sw_offset=6):
    markers = find(diff(markers[:-cycle_len]) > 0) + 1
    nm = len(markers)
    if truncate:
        iq = Truncate(iq)

    nsw = sw_len - sw_offset
    riq = empty((4, sw_count, nm), complex)
    var = 0.0
    # riq[b, n, m]
    #   b = button
    #   n = switch
    #   m = marker
    nn = size(riq)
    for i, n in enumerate(markers):
        data = reshape(iq[:, n:n+cycle_len],
            (4, sw_count, sw_len))[:, :, sw_offset:sw_len]
        riq[:, :, i] = mean(data, -1)
        var += sum(abs(data - riq[:, :, i, newaxis])**2)


    # Compute phase advance: average over each waveform, over all switches in
    # each cycle.  Here
    #   bb[m] = mean(b, n; angle(riq[b,n,m]))
    # We take means over time before converting to angle, but do the
    # remaining means as angles.
    bb = mean(mean(angle(riq), 1), 0)
    bb = bb - bb[0]
    # Compute the phase advance from position 0 and the corresponding
    # compensation required.
    mbb = exp(- 1j * bb)
    # Apply the compensation to every point
    riqc = riq * mbb

#    riqc = riq



#     var = mean(
#         abs(riqc - mean(riqc, 2)[:,:,newaxis])**2)

    var /= 4 * sw_count * nm * nsw

    dev = sqrt(var) / amin(abs(riqc))
    return mean(riqc, 2).transpose(), dev




def ReadSpikes(ioc, length=5):
#    iq, markers = GetIqSC(ioc)
    iq, markers = GetIq(ioc, 2048)
    riq, bb, riqc = NewDigestWaveform(iq, markers, sw_offset=0)
    buttons = mean(mean(mean(riqc[...,5:], 3), 2), 1)
    spikes = mean(riqc[..., :length], 2)
    return spikes / buttons[:, newaxis, newaxis]


def PlotSpikes(ioc, interval=1):
    from qt import qApp, SIGNAL
    qApp.connect(qApp, SIGNAL('lastWindowClosed()'), qApp.quit)
    pylab.figure()
    pylab.show(mainloop=False)
    while True:
        sp = ReadSpikes(ioc, 40)
        pylab.clf()
        pylab.subplot(2,1,1)
        plot_array(abs(sp.reshape((32,40))))
        pylab.subplot(2,1,2)
        plot_array(180/pi*unwrap(angle(sp.reshape((32,40)))))
#        asleep(interval)
        pylab.draw()




def DecompensateDigest(digest, compensation):
    '''Takes a digest of the form
        Y[n,b] = K[p[n,b]] C[p[n,b]] X[b]
    where
        Y[n,b]  is the reading for channel b in switch position n
        K[c]    is the current compensation setting for channel c
        C[c]    is the (modelled) gain generated by channel c
        X[b]    is the input on button b
        p[n,b]  is the channel used to process button b for switch position n
    and computes
        U[n,b] = Y[n,b] / K[p[n,b]] = C[p[n,b]] X[b] .'''

    # perms[switches] returns an (8,4) array with entries p[n,b] as required.
    p = perms[switches]
    return digest / compensation[p]


def EstimateX(decompensated):
    '''Given a decompensated digest U[n,b] returns an estimate for X
        X^[b] = mean_n U[n,b] .'''
    return mean(decompensated, 0)


def ChannelGains(U, X):
    '''Channel gains per switch.'''
    # I haven't figured out how to compute the permutation U[n, q[n,c]], so
    # instead I build it line by line here: Z[n,p[n,b]] = U[n,b] / X[b], which
    # means that Z[n,c] = U[n,q[n,c]] / X[q[n,c]].
    p = perms[switches]
    Z = zeros(shape(p), dtype=complex)
    for n in range(len(switches)):
        Z[n, p[n]] = U[n] / X
    return Z


def RelativeAngles(X):
    return (180/pi*(X[1:] - X[0]) + 180) % 360 - 180


def AbsAngleMean(X, axis=0):
    '''Returns mean of X with magnitude and angle averaged separately.'''
    mean_abs = mean(abs(X), axis=axis)
    mean_angle = mean(angle(X), axis=axis)
    return mean_abs * exp(1j*mean_angle)


def ProcessDigest(U):
    '''Given a decompensated digest U[n,b] returns the modelling errors.  By
    the model we expect that
        U[n,b] = C[p[n,b]] X[b]
    and so we estimate (assuming mean_c C[c] = 1 and p[n,:] covers all
    channels)
        X^[b] = mean_n U[n,b]
        C^[c] = mean_n U[n,q[n,c]] / X[q[n,c]]
    where q[n] is the inverse permutation to p[n] (ie q[n,p[n,b]] = b and
    p[n,q[n,c]] = c for all n, b, c).
    '''

#    X = AbsAngleMean(U)
    X = mean(U, 0)
    Z = ChannelGains(U, X)

    C = mean(Z, 0)

    return X, C, Z


def ReadChannelGains(ioc):
    '''Performs all the work of reading the current channel gains.'''

    # Y[n,b] is the button reading for button b in switch position n after
    # full FPGA processing.
    Y, _ = GetDigest(ioc)
    # K[c] is the channel compensation programmed in at the time Y was read.
    K = GetOldCompensation(ioc)
    # U = Y/K is the raw buttn readings
    U = DecompensateDigest(Y, K)
    return ProcessDigest(U)


def ReadChannelErrors(ioc):
    X, C, Z = ReadChannelGains(ioc)
    return Z - repeat(C[NewAxis, :], 8, axis=0)


def CaptureChannelErrors(ioc, count, delay):
    E = zeros((count, 8, 4), dtype=complex)
    for n in range(count):
        print '.'
        E[n, ...] = ReadChannelErrors(ioc)
        time.sleep(delay)
    return E


def ReadErrors(ioc):
    digest, my_s = GetDigest(ioc)
    print 'Deviation:', my_s
    compensation = GetOldCompensation(ioc)
    X, C, Z = ProcessDigest(DecompensateDigest(digest, compensation))
    rC = repeat(C[NewAxis,:], 8, axis=0)
    return (Z - rC) / rC, X



def PlotErrors(ioc):
    while True:
        X, C, Z = ReadChannelGains(ioc)
        clf()
        plot_complex(Z-repeat(C[NewAxis,:], 8, axis=0))
        time.sleep(1)


def DoTestSC(ioc):
    '''Reads the SC data and tries to replicate the calculation.'''

    iq, markers = GetIqSC(ioc)
    digest, my_s = GetDigest(ioc)
    m, s = DigestWaveform(iq, markers, True)

    buttons = mean(m, axis=0)
    offsets = m - buttons.reshape((1,4)).repeat(8, 0)

    print 'Deviation =', 100 * s

    compensation = GetOldCompensation(ioc)
    X, C, Z = ProcessDigest(DecompensateDigest(digest, compensation))
    print 'Inputs:', RelativeAngles(angle(X))
    print 'Scaling:', mean(abs(C))
    print 'Angles:', 180/pi*angle(C)
    print 'Magnitudes:', abs(C)

    E = Z - repeat(C[NewAxis,:], 8, axis=0)
    plot_complex(E)
    show()
    return

    subplot(2,2,1)
    plot_array(abs(iq))

    subplot(2,2,2)
    plot_array(180./pi*unwrap(angle(iq)))

    subplot(2,2,3)
    plot_complex(m)
    plot_complex(buttons, blob='o')

    subplot(2,2,4)
    plot_complex(offsets)
    show()


def NewTestSC(ioc):
    iq, markers = GetIqSC(ioc)


# SR intermediate frequency
f_if = 2 * pi * 936 / 220


def caPutArray(pv, a):
    # Oh damn.  catools.caput doesn't seem to work (only puts an array of
    # length 1 -- maybe I'm not calling it right).  Rather than investigate,
    # here's a quick and hacky workaround.
    os.system('%s -w5 -a %s %d %s >/dev/null' % (
        '/dls_sw/epics/R3.14.8.2/base/bin/linux-x86/caput',
        pv, len(a), ' '.join(map(str, a))))


Last_K = ones((8, 4), dtype=complex)


def WriteCompensation(ioc, K):
    '''Writes a new compensation matrix K to ioc.  The format of K is
        K[n,c]
    where n is the index into the switch cycle and c is the channel index.
    '''

    # The components of the two pole iir corresponding to multiplication by
    # x+iy at frequency f are (x + y/tan(f), - y/tan(f)).
    iir = int_(round_(0x10000 * array(
        [real(K) + imag(K)/tan(f_if), - imag(K)/sin(f_if)])))

    sh = (16, 4)
    full_iir = int_(array([0x10000 * ones(sh), zeros(sh)]))
    full_iir[:, switches, :] = iir

    # Make sure we don't overflow the compensator: 18 bits signed.
    assert all(abs(full_iir) < 2**17)

    # Transform into the correct form required for output.  The computed iir
    # is in the form
    #   iir[i,n,c]
    # and we need to send it over in the form (C layout)
    #   iir[n,c,i]

    output = full_iir.transpose((1, 2, 0)).ravel()
#    caPut(ioc, 'SC:SETPHASE', output)
    caPutArray('%s:%s' % (ioc, 'SC:SETCOMP_S'), output)

    global Last_K
    Last_K = K



p = perms[switches]
n = arange(len(switches))
np = (n[:, newaxis], p)

q = empty_like(p)
q[np] = arange(4)[newaxis, :]
nq = (n[:, newaxis], q)


def GlobalCompensate(Y, K):
    '''Input: Y[n,b], K[n,c].
    Output: K'[n,c].
    '''

    # Z[n,c] = Y[n, q[n,c]] / K[n, c]
    Z = Y[nq] / K
    # X^[b] = mean(n; Z[n, p[n,b]])
    X_ = mean(Z[np], 0)
    # C^[c] = mean(n; Z[n,c] / X_[q[n,c]])
    C_ = mean(Z / X_[q], 0)

    s = mean(abs(C_))
    K_ = s / C_
    return tile(K_[newaxis, :], (8, 1))




# def ChannelGains(Y, K):
#     # Z[n,b] = Y[n,b] / K[n,p[n,b]]
#     Z = Y / K[np]
#     # X^[b] = mean(n; Z[n,b])
#     X_ = mean(Z, 0)
#     # C^[c] = mean(n; Z[n,q[n,c]] / X_[q[n,c]])
#     return mean(Z[nq] / X_[q], 0)


def ChannelGains(Y, K):
    # Z[n,b] = Y[n,b] / K[n,p[n,b]]
    Z = Y / K[np]
    # X^[b] = mean(n; Z[n,b])
    X_ = mean(Z, 0)
    # C^[c] = mean(n; Z[n,q[n,c]] / X_[q[n,c]])
    return mean(Z[nq] / X_[q], 0)


# def ChannelCompensate(Y, K):
#     '''Input: Y[n,b], K[n,c].
#     Output: K'[n,c].
#     '''
#
#     # Z[n,c] = Y[n, q[n,c]] / K[n, c]
#     Z = Y[nq] / K
#     # X^[b] = mean(n; Z[n, p[n,b]])
#     X_ = mean(Z[np], 0)
#     # C^[n,c] = Z[n,c] / X_[q[n,c]]
#     C_ = Z / X_[q]
#
#     s = mean(abs(C_))
#     K_ = s / C_
#     return K_


def ChannelCompensate(Y, K):
    '''Input: Y[n,b], K[n,c].
    Output: K'[n,c].
    '''

    K = K[np]

    Z = Y / K
    K_ = mean(Z, 0) / Z
    K_ /= mean(abs(K_))

    return K_[nq]


# def ItechAbsCompensate(Y, K):
#     # Y^ = |Y|
#     Y = abs(Y)
#     # K^[n,b] = |K[n, p[n,b]]|
#     K = abs(K[np])
#
#     # G[b] = geo(n; Y^[n,b])
#     G = geo_mean(Y, 0)
#     # H[b] = geo(n; K^[n,b])
#     H = geo_mean(K, 0)
#
#     K_ = K * G / (Y * H)
#     return K_[nq]


def AbsCompensate(Y, K):
    Z = abs(Y / K[np])
    K_ = geo_mean(Z, 0) / Z
    return K_[nq]


# def AngleCompensate(Y, K):
#     Y = angle(Y)
#     K = angle(K[np])
#     Z = Y - K
#     K_ = mean(Z, 0) - Z
#
#     return exp(1j * K_[nq])


def AngleCompensate(Y, K):
    Z = Y / K[np]

    # The geometric mean has an error of 1/N turns, while the arithmetic mean
    # is in the right quadrant but is less precise numerically.
    alpha = mean(angle(Z), 0)
    beta  = angle(mean(Z, 0))

    X = alpha - pi/4 * round_(4/pi * (alpha - beta))
    K_ = X - angle(Z)

    return K_[nq]




# def ExperimentalCompensate(Y, K):
#     K_abs = AbsCompensate(Y, K)
#     K_arg = AngleCompensate(Y, K)
#     return K_abs * exp(1j * K_arg)



def ExperimentalCompensate(Y, K):
    Z = Y / K[np]
    K_abs = geo_mean(abs(Z), 0) / abs(Z)

    # The geometric mean has an error of 1/N turns, while the arithmetic mean
    # is in the right quadrant but is less precise numerically.
    alpha = mean(angle(Z), 0)
    beta  = angle(mean(Z, 0))
    X = alpha - pi/4 * round_(4/pi * (alpha - beta))
    K_arg = X - angle(Z)

    K_ = cis(K_abs, K_arg)

    return K_[nq]



def ExperimentalCompensate(Y, K):
    Z = Y / K[np]

    X_abs = geo_mean(abs(Z), 0)
    X_arg = angle(mean(Z, 0))
    X = cis(X_abs, X_arg)

    K_ = X / Z

    return K_[nq]



def ExperimentalCompensate2(Y, K):
    Z = Y / K[np]
    X = geo_mean(Z, 0)
    K_ = X / Z


    # The geometric mean has an error of 1/N turns, while the arithmetic mean
    # is in the right quadrant but is less precise numerically.
    alpha = angle(X)
    beta  = angle(mean(Z, 0))
    k = pi/4 * round_(4/pi * (alpha - beta))
    K_ *= exp(-1j * k)

    return K_[nq]



ResetIIR = True
IIR_K = None


def DoIIR(K, alpha, reset=False):
    global ResetIIR, IIR_K
    if ResetIIR:
        IIR_K = K
        ResetIIR = False
    else:
        IIR_K = (1 - alpha) * IIR_K + alpha * K

    return IIR_K


def ManualCondition(ioc, oldK=None, resetIIR=False):
    try:
        iq, m = GetIq(ioc, 2048)
    except AssertionError:
        print 'GetIq failed'
        return

    Y, d = DigestWaveform(iq, m)

    print 'Deviation %.2f %%' % (100 * d)
    if d > 0.02:
        print '  Deviation too high'
        return

    if oldK is None:
        oldK = Last_K


#    K = GlobalCompensate(Y, oldK)
#    K = ChannelCompensate(Y, oldK)
#    K = ItechAbsCompensate(Y, oldK)
    K = ExperimentalCompensate(Y, oldK)
#    K = DoIIR(K, 0.1, resetIIR)
    WriteCompensation(ioc, K)


    if not all(int_(round_(8/pi * mean(angle(K), 1))) == 0):
        print int_(round_(8/pi * mean(angle(K), 1)))
#        assert False

    C = 1/mean(K, 0)
#    niq = iq / mean(iq, 1)[:, newaxis]
    niq = iq / mean(iq)

    pylab.clf()

    pylab.subplot(2,3,1)
    plot_array(abs(niq))
    pylab.title('IQ magnitudes')

    pylab.subplot(2,3,2)
    plot_array(180/pi*unwrap(angle(niq / mean(niq, 1)[:, newaxis])))
    pylab.title('IQ phases')

    pylab.subplot(2,3,3)
#     pylab.plot(180/pi*unwrap(angle(mean(niq, 1))), 'o')
#     pylab.xlim(-0.5, 3.5)
    plot_complex(mean(niq, 1), plot_circle=True, blob='o')
    pylab.axis('equal')
    pylab.title('IQ relative phase')

    pylab.subplot(2,2,3)
    plot_complex(K, True)
#    plot_complex(mean(C, 0) / mean(abs(C)), blob='o')
    plot_complex(mean(abs(C)) / C, blob='o')
    pylab.axis('equal')
    pylab.title('Compensations')

#     pylab.subplot(4,2,6)
#     Ym = abs(Y/mean(Y))
#     plot_array(Ym - mean(Ym, 0))
#    plot_complex(mean(C, 0) / mean(abs(C)), blob='o')

#     pylab.subplot(4,2,8)
#     pylab.plot(180/pi*unwrap(bb))

    for i in range(2):
        for j in range(2):
            pylab.subplot(4, 4, 11 + i + 4*j)
            k = i + 2*j
            plot_complex(K[:, k])
            plot_complex(array([1/C[k]]), blob='o')

    return Y, K, oldK


def RunConditioning(ioc, interval=1):
    WriteCompensation(ioc, Last_K)

    from qt import qApp, SIGNAL
    qApp.connect(qApp, SIGNAL('lastWindowClosed()'), qApp.quit)
    pylab.figure()
    pylab.show(mainloop=False)
    while True:
        ManualCondition(ioc)
        pylab.draw()
        asleep(interval)


def ZeroOffsets(ioc):
    for a in 'XY':
        current = caGet(ioc, 'SA:%s' % a)
        bcd = caGet(ioc, 'CF:BBA_%s_S' % a)
        caPut(ioc, 'CF:BBA_%s_S' % a, current + bcd)

if __name__ == '__main__':
    if len(sys.argv) not in [2, 3]:
        print 'Usage: %s <ioc> [<png-file>]' % sys.argv[0]
        sys.exit(1)

#    DoTestSC(sys.argv[1])
#    PlotErrors(sys.argv[1])
#    NewTestSC(sys.argv[1])
#    start_gui(lambda: PlotSpikes(sys.argv[1]))
    start_gui(lambda: RunConditioning(sys.argv[1]))

#     if len(sys.argv) > 2:
#         savefig(sys.argv[2] + '.png')
#
#     show()

