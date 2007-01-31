#!/usr/bin/env python2.4

from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools

import re
import sys, os
import time


def all(l):
    '''Returns true iff every value in l is true.'''
    for i in l:
        if not i:
            return False
    return True

def any(l):
    '''Returns true iff every value in l is true.'''
    for i in l:
        if i:
            return True
    return False


def GetAll(pvs):
    sl, vl = catools.caget(pvs, timeout = 5, datatype = catools.dbr_string)
    result = [v.value for v in vl]
    ok = True
    for pv, s in zip(pvs, sl):
        if s != catools.ECA_NORMAL:
            print >>sys.stderr, pv, catools.ca_message(s)
            ok = False
    return ok, result
    

def PutAll(pvs, values):
    sl = catools.caput(pvs, values, timeout = 5, datatype = catools.dbr_string)
    ok = True
    for pv, s in zip(pvs, sl):
        if s != catools.ECA_NORMAL:
            print >>sys.stderr, pv, catools.ca_message(s)
            ok = False
    return ok
#    return all([s == catools.ECA_NORMAL for s in sl])

    
def SafePutAll(pvs, value):
    values = [value for pv in pvs]
    if not PutAll(pvs, values):
        print 'Failed'
        return False
    ok, written = GetAll(pvs)
    if not ok or written != values:
        print 'Read-back failed!', ok, written
        return False
    return True



TrigModePvs = ['SR%02dC-DI-EVR-01:TRIG:MODE' % (c+1) for c in range(24)]
LiberaPvs = ['SR%02dC-DI-EBPM-%02d:CF:SYNC_S' % (c+1, n+1)
    for c in range(24) for n in range(7)] + ['SR21C-DI-EBPM-08:CF:SYNC_S']

print
print 'Stopping trigger:'
SafePutAll(TrigModePvs, 'Synchronised')

print
print 'Waiting a little'
time.sleep(2)

print
print 'Sending synchronisation event'
SafePutAll(LiberaPvs, 'Synchronise')

print
print 'Waiting for Liberas to be ready'
time.sleep(4)

print
print 'Sending synchronise event'
catools.caput('LI-TI-MTGEN-01:SR-DI', 1, timeout = 5)
catools.caput('LI-TI-MTGEN-01:SR-DI', 0, timeout = 5)

print
print 'Waiting for synchronise event'
time.sleep(3)


# Always try to restore normal operation at the end.
SafePutAll(TrigModePvs, 'Normal')
