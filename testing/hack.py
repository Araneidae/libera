#from __future__ import division

from numpy import *


class Test:
    def __init__(self):
        self.alpha = 0.03
        self.beta = 0.8
        self.r0 = 0.5
        self.r1r = 0.5
        self.r1i = 0.5
        
    
    def UpdateValues(self):
        # From root positions compute the corresponding actual roots
        beta_ = 1 - self.beta
        r0 = self.beta + self.r0 * beta_
        r1 = self.beta + self.r1r * beta_ + beta_ * self.r1i * 1j

        self.BasePoles = array([r0, r1, r1.conjugate()])
        self.R = poly1d(poly(self.BasePoles))
        
        self.A = poly1d([1, -1]) * poly1d([1, -self.beta])
        print repr(self.R - poly1d([1, -1]) * self.A)
        print self.alpha, type(self.alpha)
        print poly1d.__module__
        self.alpha = 1.0
        self.B = (self.R - poly1d([1, -1]) * self.A) / self.alpha

t = Test()
t.UpdateValues()
