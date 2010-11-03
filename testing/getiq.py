#!/usr/bin/env python2.4

if __name__ == '__main__':
    from pkg_resources import require
    require('cothread')

    import os
    os.environ.setdefault('EPICS_CA_MAX_ARRAY_BYTES', '1000000')


from cothread import *
from cothread.catools import *
from numpy import *

import time


def caGet(ioc, pv):
    return caget('%s:%s' % (ioc, pv))

def caPut(ioc, pv, value):
    caput('%s:%s' % (ioc, pv), value)

def caGetArray(ioc, pvs):
    return caget(['%s:%s' % (ioc, pv) for pv in pvs])


def BlockGet(pvs, total_length, window_length, set_offset):

    pv_count = len(pvs)

    result = zeros((pv_count, total_length), dtype = int32)
    seen = zeros(pv_count, dtype = bool)
    ready = Event()

    timestamp = [None]

    offset = 0
    capture_length = min(window_length, total_length)
    set_offset(offset)

    def OnUpdate(value, index):
        if seen[index]:
            ready.SignalException(
                Exception('Multiple updates on same record!'))
            return

        if timestamp[0] is None:
            timestamp[0] = value.timestamp
        elif timestamp[0] != value.timestamp:
            ready.SignalException(Exception('Inconsistent timestamps'))
            return

        result[index, offset : offset + window_length] = value[:capture_length]
        seen[index] = True
        if all(seen):
            ready.Signal()


    subscriptions = camonitor(pvs, OnUpdate,
        format = FORMAT_TIME, all_updates = True)
    ready.Wait()

    while offset + window_length < total_length:
        seen[:] = False
        offset += window_length
        capture_length = min(window_length, total_length - offset)
        set_offset(offset)
        ready.Wait()

    for subscription in subscriptions:
        subscription.close()

    return result


def SetOffset(ioc, offset):
    caPut(ioc, 'TT:OFFSET_S', offset)
    while caGet(ioc, 'TT:OFFSET_S') != offset:
        Sleep(0.01)


def GetIq(ioc, request_length):
    # Interrogate the default window length and set up trigger capture and
    # readout windows.
    window_length = caget(ioc + ':TT:WFAI.NELM', datatype = int)
    caPut(ioc, 'TT:CAPLEN_S', request_length)
    caPut(ioc, 'TT:LENGTH_S', window_length)

    # Perform the trigger capture handshake.
    caPut(ioc, 'TT:READY', 0)
    while caGet(ioc, 'TT:READY') != 0:
        time.sleep(0.01)
    caPut(ioc, 'TT:ARM', 1)
    while caGet(ioc, 'TT:READY') == 0:
        Sleep(0.01)

    captured = caGet(ioc, 'TT:CAPTURED')

    start = time.time()
    if True:
        wfs = BlockGet(
            ['%s:TT:WF%s%s' % (ioc, button, iq)
                for button in 'ABCD' for iq in 'IQ'],
            captured, window_length, lambda offset: SetOffset(ioc, offset))
    else:
        wfs = zeros((8, captured), dtype = int32)
        offset = 0
        while offset < captured:
            SetOffset(ioc, offset)
            a = caget(
                ['%s:TT:WF%s%s' % (ioc, button, iq)
                    for button in 'ABCD' for iq in 'IQ'],
                format = FORMAT_TIME,
                count = min(captured - offset, window_length))
            wfs[:, offset : offset + window_length] = array(a).reshape(8, -1)
            offset += window_length

    duration = time.time() - start
    print 'took', duration, 'seconds'
#     times = [wf.timestamp for wf in wfs]
#
#     for t in times[1:]:  assert t == times[0]

    wfs.shape = (4, 2, -1)
    return wfs[:, 0, :] + 1j * wfs[:, 1, :], wfs[0, 0, :] & 1



if __name__ == '__main__':
    from sys import argv
    iqs = GetIq(argv[1], int(argv[2]))
#    iq = GetIq(argv[1], int(argv[2]))
    print [(iq.shape, iq.dtype) for iq in iqs]

