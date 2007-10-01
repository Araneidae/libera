#!/usr/bin/env python2.4

#from __future__ import division

from pkg_resources import require as Require
Require('matplotlib')
Require('dls.ca2')
Require('dls.thread')

from dls import thread
import pylab
import qt
from numpy import *



# turn of interactive mode (don't draw until called)
pylab.ioff()


def BuildGraph(fig, *sliders):
    '''Builds a graph with the given list of sliders connected: each slider
    callback is called as the corresponding slider is moved.

    Note that the top level handle returned must be retained by the caller!
    '''
    top = qt.QHBox()
    # put figure in box
    fig.canvas.parent().parent().reparent(top, qt.QPoint(0,0))

    # add scroll bars
    for slider in sliders:
#        scroll = qt.QScrollBar(top)
        slider.slider.reparent(top, qt.QPoint(0,0))
#         qt.QObject.connect(
#             slider.slider, qt.SIGNAL("valueChanged(int)"), slider.Update)

    top.resize(400, 300)
    top.show()
    return top


# class Slider:
#     def __init__(self, initial, min, max, OnUpdate):
#         self.slider = qt.QScrollBar()
# 
#     def Value(self):
#         return self.initial # translated
# 
#     def Update(self)


def History(history, new):
    return concatenate(([new], history[:-1]))

def ImpulseResponse(B, A, count):
    '''Returns the impulse response of a system of the form
                                    n
        B[0] + B[1] z + ... + B[n] z
        -----------------------------
                                    m
        A[0] + A[1] z + ... + A[m] z

    We do this by running the corresponding IIR on an impulse input after
    transforming the system into the corresponding filter.
    '''

    assert A.order >= B.order, 'Futuristic filter'

    An = A.c
    Bn = concatenate((B.c, zeros(A.order - B.order)))
    A0 = An[0]
    Bn = Bn/A0
    An = An[1:]/A0

    incoming = concatenate(([1], zeros(A.order)))
    history = zeros(A.order)

    result = zeros(count)
    for i in range(count):
        next = dot(incoming, Bn) - dot(history, An)
        result[i] = next
        incoming = History(incoming, 0)
        history = History(history, next)

    return result



class Line:
    def __init__(self, Values, *Shape, **kwargs): #, plot = pylab.plot):
        self.Values = Values
        self.line, = pylab.plot(*self.Values() + Shape, **kwargs)
#        self.line, = pylab.plot(markersize=20, *self.Values() + Shape)

    def Update(self):
        self.line.set_data(*self.Values())



class TestGraph:
    def __init__(self):
        self.lines = []
        
        # draw line
        self.fig = pylab.figure()

        pylab.subplot(2,2,1)
        pylab.title('Poles')
        unit = exp(1j * linspace(0, 2*pi, 100))
        pylab.plot(unit.real, unit.imag, ':')
        pylab.axis('equal')


        self.sliders = [
            self.MakeSlider('alpha', 0.03,   0,   0.1),
            self.MakeSlider('beta',  0.8,    0.8, 1),
#             self.MakeSlider('b0',    0.3,    0,   1),
#             self.MakeSlider('b1',    0.14,   0,   0.5),
#             self.MakeSlider('b2',   -0.41,  -1,   0),
            self.MakeSlider('r0',   0.5,    -0.5,      1),
            self.MakeSlider('r1r',  0.5,   -1.1,    1),
            self.MakeSlider('r1i',  0.5,    0,      2)
        ]
        self.UpdateValues()

        self.AddLine(self.Poles, 'D')
        for i in range(3):
            self.AddLine(self.PoleN(i))
#            self.AddLine(self.PoleN(i), '.')
        
        self.AddLine(self.Zeros, 'o')
        pylab.axis([0.7, 1.1, -0.4, 0.4])

        pylab.subplot(4,2,2)
        self.AddLine(self.SpectrumAbs)  #, plot = pylab.semilog)

        pylab.subplot(4,2,4)
        self.AddLine(self.SpectrumPhase)  #, plot = pylab.semilog)

        pylab.subplot(2,2,3)
        self.AddLine(self.Impulse)

        pylab.subplot(2,2,4)
        self.AddLine(self.Controller)
        
        self.top = BuildGraph(self.fig, *self.sliders)



    def AddLine(self, *args, **kargs):
        self.lines.append(Line(*args, **kargs))


    def MakeSlider(self, name, initial, low=0.0, high=1.0):
        setattr(self, name, initial)

        class Slider:
            def __init__(self, target):
                self.target = target
                self.slider = qt.QScrollBar(None)
            def Update(self, val):
                setattr(self.target, name, low + (high - low) * val / 100.)
                self.target.UpdateLines()

        Slider = Slider(self)
        Slider.slider.setValue(
            100. * (initial - low) / (high - low))
        qt.QObject.connect(
            Slider.slider, qt.SIGNAL("valueChanged(int)"),
            Slider.Update)
        
        return Slider

#         class Slider:
#             def __init__(sself):
#             def Value(sself):
#                 return ...
#             def Update(sself):
#                 self.UpdateLines()
# 
#         return Slider()

        

    def UpdateValues(self):
        # From root positions compute the corresponding actual roots
        beta_ = 1 - self.beta
        r0 = self.beta + self.r0 * beta_
        r1 = self.beta + self.r1r * beta_ + beta_ * self.r1i * 1j

        self.BasePoles = array([r0, r1, r1.conjugate()])
        self.R = poly1d(poly(self.BasePoles))
        
        self.A = poly1d([1, -1]) * poly1d([1, -self.beta])
        self.B = (self.R - poly1d([1, -1]) * self.A) / self.alpha
        print self.B

# 
#         
#         self.b = array([self.b0, self.b1, self.b2])
#         print 'alpha = %(alpha)g, beta = %(beta)g, b = %(b)s' \
#             % self.__dict__,
#         self.A = poly1d([1, -1]) * poly1d([1, -self.beta])
#         self.B = poly1d(self.b)
#         self.R = poly1d([1, -1]) * self.A + self.alpha * self.B

        self.PoleArray = self.PoleTrajectories()

        
    def UpdateLines(self):
        self.UpdateValues()
        for line in self.lines:
            line.Update()
        self.fig.canvas.draw()


    # Curves

    def PoleTrajectories(self):
        n = 200
        poles = empty((3, n-1), dtype = complex)
        for i, alpha in enumerate(linspace(0, 0.1, n)[1:]):
            R = poly1d([1, -1]) * self.A + alpha * self.B
            poles[:,i] = sorted(roots(R), key = imag) 
        return poles

    def PoleN(self, n):
        def PoleArray():
            return self.PoleArray[n].real, self.PoleArray[n].imag
        return PoleArray

    
    def Poles(self):
        return self.BasePoles.real, self.BasePoles.imag
        
        poles = roots(self.R)
        print poles
        print max(abs(poles))
        return poles.real, poles.imag

    def Zeros(self):
        zeros = array([1, self.beta])
        return zeros.real, zeros.imag

    def SpectrumAbs(self):
        ff = linspace(0, 0.5, 1000)[1:]
        e_ff = exp(2j * pi * ff)
        logs = 20 * log10(abs(self.A(e_ff) / self.R(e_ff)))
        good = nonzero(isfinite(logs))
        return ff[good], logs[good]

    def SpectrumPhase(self):
        ff = linspace(0, 0.5, 1000)[1:]
        e_ff = exp(2j * pi * ff)
        return ff, 180/pi * unwrap(angle(self.A(e_ff) / self.R(e_ff)))

    def Controller(self):
        n = 20
        return arange(n), ImpulseResponse(self.B, self.A, n)

    def Impulse(self):
        n = 200
        return arange(n), ImpulseResponse(self.alpha * self.A, self.R, n)



graph = TestGraph()
# graph2 = TestGraph()
thread.exec_loop()
