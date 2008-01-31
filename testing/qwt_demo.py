#!/usr/bin/env python2.4

from pkg_resources import require

require('dls.ca2')
require('dls.green')

from dls import green

from qt import *
# convenience interface to qwt
from qwt.qplt import *
from numpy import *
# fix for qwt requiring old Numeric
from Numeric import array as old

q = QApplication([])
x = linspace(0, 2*pi, 100)

c1 = Curve(old(x), old(sin(x)), Pen(Red), "Sin")
c2 = Curve(old(x), old(cos(x)), Pen(Blue), "Cos")
p1 = Plot(c1, c2, "Chart Title")

QObject.connect(q, SIGNAL("lastWindowClosed()"), qApp.quit)


green.start_gui()

