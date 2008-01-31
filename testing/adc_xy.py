#!/usr/bin/env python2.4

import sys, os

from pkg_resources import require

require('cothread')
require('matplotlib')

import cothread
from cothread import catools
import pylab
import numpy




class Monitor:
    def __init__(self, ioc, process):
        self.process = process
        self.raw = numpy.empty((4, 1024))
        self.updated = numpy.zeros(4, dtype=bool)
        self.timestamps = numpy.zeros(4)
        catools.camonitor(
            ['%s:FT:RAW%d' % (ioc, n+1) for n in range(4)],
            self.on_update_raw,
            format = catools.FORMAT_TIME)
        self.permutation = numpy.zeros(4, dtype = int)
        catools.camonitor('%s:FT:PERM' % ioc, self.on_update_perm)

    def on_update_perm(self, value):
        self.permutation[:] = value
        
    def on_update_raw(self, value, index):
        self.raw[index] = value
        self.updated[index] = True
        self.timestamps[index] = value.timestamp
        
        if self.updated.all() and self.permutation.any():
            timestamp = self.timestamps[0]
            if (self.timestamps == timestamp).all():
                self.updated[:] = False
                self.process(self.raw, self.permutation, timestamp)
            else:
                pass
#                print 'Inconsistent timestamps', self.timestamps - timestamp

class Plot:
    def __init__(self, axes=None, do_plot=pylab.plot, subplot=None):
        self.axes = axes
        self.subplot = subplot
        self.do_plot = do_plot
        self.lines = None

    def plot(self, values):
        if self.lines:
            for line, value in zip(self.lines, values):
                line.set_ydata(value)
        else:
            if self.subplot:
                pylab.subplot(*self.subplot)
            self.lines = self.do_plot(values.T)
            if self.axes:
                pylab.axis(self.axes)


def RawToButton_old(raw, permutation):
    '''Simple conversion
    '''
    rp = raw[permutation]
    return abs(
        rp[:, 0::4] - rp[:, 2::4] + 1j * (rp[:, 1::4] - rp[:, 3::4]))


F_IF = 936. / 220. - 4.     #   14/55 = 1/4 + 1/220

F_IF = 0.25


def ConvolveSequence(s):
    result = numpy.array(s[0], dtype = float)
    for a in s[1:]:
        result = numpy.convolve(result, numpy.array(a, dtype = float))
    return result


def BuildFilter(theta, N):
#     filter = numpy.array([1, 1], dtype = float)     # z + 1
#     for n in range(N):
#         filter = numpy.convolve(filter,
#             numpy.array([1, 2 * numpy.cos((n+1) * theta), 1]))
    filter = ConvolveSequence(
        [[1, 1]] + [
            [1, 2 * numpy.cos((n+1) * theta), 1] for n in range(N)])
    return filter / sum(filter)

low_pass = BuildFilter(numpy.pi / 6, 2)

# low_pass = numpy.ones(4) / 4.



def RawToButton(raw, permutation):
    rotate = numpy.exp(-1j * 2 * numpy.pi * F_IF * numpy.arange(1024))
    raw_rotated = rotate * (raw - numpy.mean(raw, 1)[:, None])
#    raw_rotated = rotate * raw

#     for row in raw_rotated:
#         row[:] = numpy.convolve(row, low_pass, mode = 'same')

    plot_fft.plot(abs(numpy.fft.fft(raw_rotated)))
#     plot_fft.plot(abs(numpy.fft.fft(
#         numpy.concatenate((low_pass, numpy.zeros(1024-8))))))

    return abs(raw_rotated[:,::4])

K_X = 21.6
K_Y = 14.6
K = numpy.array([K_X, K_Y])
    

def ButtonToXY(buttons):
    s = buttons.sum(0)
    maxs = max(s)
    xym = numpy.array([
        [1, -1, -1, 1],     # X = A - B - C + D
        [1, 1, -1, -1]])    # Y = A + B - C - D
    xy = numpy.dot(xym, buttons) / s
    xy[s[None, :].repeat(2,0) < maxs/2] = numpy.nan
    return K[:, None] * xy
    

def ProcessRaw(raw, permutation, timestamp):
    buttons = RawToButton(raw, permutation)
    plot_buttons.plot(buttons)

#    plot_fft.plot(abs(numpy.fft.fft(raw)))

    xy = ButtonToXY(buttons)
    plot_xy.plot(xy)
    
    pylab.draw()

        
cothread.iqt()

ioc = sys.argv[1]

# raw = numpy.array(catools.caget(
#     ['%s:FT:RAW%d' % (ioc, n+1) for n in range(4)]))

pylab.ioff()
pylab.figure()
pylab.show()

plot_buttons = Plot(
    axes = [0, 255, 1e1, 1e5], subplot = (2, 2, 1),
    do_plot = pylab.semilogy)
plot_fft = Plot(subplot = (2, 2, 2), do_plot = pylab.semilogy)
plot_xy = Plot(
    axes = [0, 255, -1, 1], subplot = (2, 1, 2))

m = Monitor(ioc, ProcessRaw)

cothread.WaitForQuit()
