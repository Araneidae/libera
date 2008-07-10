#!/bin/env python2.4

import sys
site = "/dls_sw/tools/python2.4/lib/python2.4/site-packages/"
eggs = ["dls.ca2-2.16-py2.4.egg",
        "dls.thread-1.15-py2.4.egg",
        "matplotlib-0.91.1-py2.4-linux-i686.egg"]
for egg in eggs:
    sys.path.append(site + egg)

import iqt
from qt import *
from pylab import *
from numpy import *
import dls.thread as thread
from dls.ca2.catools import *

class Gui(QWidget):

    "Make GUI with toolbar and plot window"
    
    def __init__(self):
        QWidget.__init__(self)
        self.setCaption("TMBF")
        toolbar = QWidget(self)
        QHBoxLayout(toolbar)
        layout = QVBoxLayout(self)
        layout.addWidget(toolbar, 0)
        panel = QHBox(self)
        layout.addWidget(panel, 1)
        # create an embedded figure with animation support
        self.fig = thread.embedded_figure(panel)
        combo = QComboBox(toolbar)
        toolbar.layout().addWidget(combo)
        toolbar.layout().addStretch()
        combo.insertStrList(["X PLANE", "Y PLANE"])
        QObject.connect(combo, SIGNAL("activated(int)"), self.on_activated)
    
    def on_activated(self, num):
        num = num + 1
        store_.switch("SR21C-DI-TMBF-%02d:ADC_MINBUF_R" % num)
        calc_.switch("SR21C-DI-TMBF-%02d:ADC_MAXBUF_R" % num)
    
def calc(name):
    "calculates difference and plots"
    while True:
        mon = camonitor(name, thread.thread_monitor())
        lines = None
        while True:
            # wait for new message
            event = thread.Yield()
            # check for new channel
            if type(event) is str:
                name = event
                break
            value = event.dbr.value
            max_value = value
            x = arange(len(value))
            if lines is None:
                subplot(2,1,1)
                lines = plot(x, min_value, 'r',
                             x, max_value, 'b')
                hold(False)
                subplot(2,1,2)
                lines2 = plot(x, min_value - max_value, 'g')
                hold(False)
                gcf().canvas.animated = lines + lines2
                draw()
            else:
                lines[0].set_data(x, min_value)
                lines[1].set_data(x, max_value)
                lines2[0].set_data(x, min_value - max_value)
                gcf().canvas.animate()
        unsubscribe(mon)

def store(name):
    "stores first waveform"
    while True:
        mon = camonitor(name, thread.thread_monitor())
        while True:
            event = thread.Yield()
            if type(event) is str:
                name = event
                break
            min_value[:] = event.dbr.value
        unsubscribe(mon)

if __name__ == "__main__":
    ioff()
    thread.install_threads()
    QObject.connect(qApp, SIGNAL("lastWindowClosed()"), qApp.quit)
    min_value = zeros(2048)
#     store_ = thread.thread_new(store, "SR21C-DI-TMBF-01:ADC_MINBUF_R")
#     calc_  = thread.thread_new(calc,  "SR21C-DI-TMBF-01:ADC_MAXBUF_R")
    store_ = thread.thread_new(store, "TS-DI-EBPM-01:FR:WFX")
    calc_  = thread.thread_new(calc, "TS-DI-EBPM-01:FR:WFY")
    gui = Gui()
    gui.show()
    qApp.exec_loop()
