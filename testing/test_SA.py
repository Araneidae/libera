#!/usr/bin/env python2.4

from __future__ import division

import sys, os
import datetime

from pkg_resources import require

require('dls.ca2')
require('dls.green')

from dls.ca2.catools import *
from dls import green

install_sleep(green.co_sleep)

last_stamp = None
ticks = 0

def monitor_callback(pv):
    global last_stamp, ticks
    if last_stamp:
        diff = pv.dbr.stamp - last_stamp
        if diff < datetime.timedelta(0, 0, 90000)  or  \
           diff > datetime.timedelta(0, 0, 110000):
            if ticks > 0:
                print ticks, ':',
                ticks = 0
            print diff.microseconds / 1000
        else:
            ticks += 1
    last_stamp = pv.dbr.stamp


camonitor(sys.argv[1], monitor_callback, datatype = dbr_time_float)

while True:
    ca_pend_event(1e-9)
    green.tick()

