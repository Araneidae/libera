#!/usr/bin/env python2.4

'''Simple tool to graphically monitor Signal Conditioning.'''

from __future__ import division

import sys, os

from pkg_resources import require

require('dls.ca2==2.16')
require('dls.thread==1.10')
require('matplotlib==0.90.1')

os.environ['EPICS_CA_MAX_ARRAY_BYTES'] = '1000000'
from dls.ca2 import catools
from dls import thread
import pylab
from numpy import *

import qt
import qwt
import iqt


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


PHASE_UNITY = 0x10000
F_IF = 2 * pi * (936 % 220) / 220



class Monitor:
    IQ_names = ['WF%s%s' % (b, i) for b in 'ABCD' for i in 'IQ']
    
    def __init__(self, ioc):
        self.ioc = ioc
        
        self.values = {}
        self.stamps = set()
        
        self.monitor('DONE', self.Done)
        self.monitor('COMP')

        map(self.monitor, self.IQ_names)


    def monitor(self, pv, action = None, datatype = catools.dbr_time_long):
        if action is None:
            def action(value):
                self.values[pv] = array(value.dbr.value)
                self.stamps.add(value.dbr.stamp)
        catools.camonitor(
            '%s:SC:%s' % (self.ioc, pv), action, datatype = datatype)
            

    def Done(self, value):
        self.stamps.add(value.dbr.stamp)

        if len(self.stamps) == 1:
            try:
                rawK = self.values['COMP']
                rawIQ = array([self.values[iq] for iq in self.IQ_names])
            except KeyError:
                print '  Incomplete data set'
            else:
                self.Process(rawK, rawIQ)

        else:
            print '  Inconsistent timestamps'

        self.stamps = set()


    def Process(self, rawK, rawIQ):
        raw = rawK.reshape((8, 4, 2))
        K = (raw[..., 0] + exp(-1j*F_IF) * raw[..., 1]) / PHASE_UNITY
        C = 1 / mean(K, 0)

        rawIQ = rawIQ.reshape((4, 2, -1))
        IQ = rawIQ[:, 0, :] + 1j * rawIQ[:, 1, :]

        nIQ = IQ / mean(IQ)
        t = arange(nIQ.shape[1])

        self.Plot(nIQ, K, C)


    def Plot(self, nIQ, K, C):
        pylab.clf()

        pylab.subplot(2,3,1)
        plot_array(abs(nIQ))
        pylab.title('IQ magnitudes')

        aIQ = unwrap(angle(nIQ / mean(nIQ, 1)[:, newaxis]))
        pylab.subplot(2,3,2)
        plot_array(180/pi * aIQ)
        pylab.title('IQ phases')
        
        pylab.subplot(2,3,3)
        plot_complex(mean(nIQ, 1), plot_circle=True, blob='o')
        pylab.axis('equal')
        pylab.title('Button inputs')

        pylab.subplot(2,2,3)
        plot_complex(K, True)
        plot_complex(mean(abs(C)) / C, blob='ro')
        pylab.axis('equal')
        pylab.title('Compensations')

        for i in range(2):
            for j in range(2):
                pylab.subplot(4, 4, 11 + i + 4*j)
                k = i + 2*j
                plot_complex(K[:, k])
                plot_complex(array([1/C[k]]), blob='ro')

        pylab.draw()
            
    
    
if __name__ == '__main__':
    if len(sys.argv) not in [2, 3]:
        print 'Usage: %s <ioc> [<png-file>]' % sys.argv[0]
        sys.exit(1)

    qt.qApp.connect(qt.qApp, qt.SIGNAL('lastWindowClosed()'), qt.qApp.quit)

    pylab.ioff()
    pylab.figure()
    pylab.show()
    
    Monitor(sys.argv[1])
#    green.start_gui()
    thread.exec_loop()
