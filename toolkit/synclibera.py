#!/usr/bin/env python

import sys
import os.path
import optparse
import time

from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools



# We have three separate synchronisation groups: LB, BS and TS.

LB_BPMS = \
    ['LB-DI-EBPM-%02d' % (num + 1) for num in range(7)] + \
    ['BR%02dC-DI-EBPM-%02d' %
        (cell + 1, 1 + (5*cell + (cell+1)//2 + num - 3) % 22)
        for cell in range(4)
        for num in range(5 + (cell+1) % 2)]

BS_BPMS = \
    ['BS-DI-EBPM-%02d' % (num + 1) for num in range(7)]

SR_BPMS = \
    ['SR%02dC-DI-EBPM-%02d' % (cell + 1, num + 1)
        for cell in range(24) for num in range(7)] + \
    ['SR21C-DI-EBPM-08']

TS_BPMS = ['TS-DI-EBPM-%02d' % (num + 1) for num in range(6)]

LB_EVRS = \
    ['LI-DI-EVR-01'] + \
    ['BR%02dC-DI-EVR-01' % (num+1) for num in range(4)]

BS_EVRS = \
    ['BS-DI-EVR-01']

SR_EVRS = \
    ['SR%02dC-DI-EVR-01' % (num+1) for num in range(24)]

TS_EVRS = ['TS-DI-EVR-01']

SR_TRIGGER = 'LI-TI-MTGEN-01:SR-DI'
LB_TRIG = 'LI-TI-MTGEN-01:LB-DI-MODE'
BS_TRIG = 'LI-TI-MTGEN-01:BS-DI-MODE'


def PVs(bases, pv):
    return ['%s:%s' % (base, pv) for base in bases]

def caput(pvs, value, datatype=None):
    catools.caput(pvs, [value for _ in pvs], datatype = datatype, timeout = 5)

def caget(pvs, datatype=None):
    return catools.caget(pvs, datatype = datatype, timeout = 5)


def CheckReadbacks(bpms, pv, expected, datatype=None):
    for bpm, s in zip(bpms, caget(PVs(bpms, pv), datatype = datatype)):
        if s.value != expected:
            print bpm, s.value

def GetActiveBpms(bpms):
    print 'Checking active BPMs'
    enabled_list = catools.caget(
        PVs(bpms, 'CF:ENABLED_S'), timeout = 5, throw = False)
    result = []
    for bpm, enabled in zip(bpms, enabled_list):
        if enabled.status != catools.ECA_NORMAL:
            print ' ', bpm, 'not responding'
        elif not enabled.value:
            print ' ', bpm, 'disabled'
        else:
            result.append(bpm)
    return result
    

def GetActiveEvrs(evrs):
    print 'Checking active EVRs'
    enabled_list = catools.caget(
        PVs(evrs, 'TRIG:MODE'), timeout = 5, throw = False)
    result = []
    for evr, enabled in zip(evrs, enabled_list):
        if enabled.status != catools.ECA_NORMAL:
            print ' ', evr, 'not responding'
        else:
            result.append(evr)
    return result


def GlobalSystemTrigger():
    catools.caput(SR_TRIGGER, 1)
    catools.caput(SR_TRIGGER, 0)

def LabSystemTrigger():
    catools.caput('TS-DI-EVR-01:TS-RESET.OUT2', 1)

class GlobalMachineTrigger:
    def __init__(self, triggers):
        self.triggers = triggers
    def Disable(self):
        caput(self.triggers, 'Off', catools.dbr_string)
    def Enable(self):
        caput(self.triggers, 'Every shot', catools.dbr_string)

class LabMachineTrigger:
    def Disable(self):
        catools.caput(
            'TS-DI-EVR-01:TRIG:MODE', 'Synchronised',
            datatype = catools.dbr_string)
    def Enable(self):
        catools.caput(
            'TS-DI-EVR-01:TRIG:MODE', 'Normal',
            datatype = catools.dbr_string)
    

def SynchroniseSystemClocks(bpms, evrs, trigger):
    # First put all EVRs into Synchronised mode
    print 'Synchronising system clocks'
    print '  Switching to Synchronised mode'
    caput(PVs(evrs, 'TRIG:MODE'), 'Synchronised', catools.dbr_string)
    CheckReadbacks(evrs, 'TRIG:MODE', 'Synchronised', catools.dbr_string)
    time.sleep(5)
    
    print '  Enabling SC trigger'
    caput(PVs(bpms, 'CK:SC_SYNC_S'), 0)
    time.sleep(2)
    CheckReadbacks(bpms, 'CK:SC_SYNC', 'Waiting Trigger', catools.dbr_string)
    
    print '  Triggering SC synchronisation'
    trigger()
    time.sleep(5)
    
    print '  Restoring Normal mode'
    caput(PVs(evrs, 'TRIG:MODE'), 'Normal', catools.dbr_string)

    

def SynchroniseMachineClocks(bpms, triggers):
    print 'Synchronising machine clocks'

    # First disable all the triggers.
    print '  Disabling events'
    triggers.Disable()
    time.sleep(5)
    
    print '  Arming MC trigger'
    caput(PVs(bpms, 'CK:MC_SYNC_S'), 0)
    time.sleep(5)
    CheckReadbacks(bpms, 'CK:MC_SYNC', 'Waiting Trigger', catools.dbr_string)
    
    print '  Triggering MC synchronisation'
    triggers.Enable()
    time.sleep(2)


def FindCommonTime(times):
    d = {}
    for time in times:
        if time not in d:
            d[time] = 0
        d[time] += 1
        
    c = 0
    for time, count in d.iteritems():
        if count > c:
            t = time
            c = count
    return t
        

def AnalyseTimes(location, bpms, times):
    common_time = FindCommonTime(times)
    
    print '%s:' % location, common_time
    for bpm, time in zip(bpms, times):
        if time != common_time:
            print ' ', bpm, time

        

def CheckSystemClocks(bpms, evrs, trigger):
    print 'Checking system clocks'
    print '  Switching to Synchronised mode'
    caput(PVs(evrs, 'TRIG:MODE'), 'Synchronised', catools.dbr_string)
    time.sleep(5)
    print '  Triggering'
    trigger()
    time.sleep(3)
    mcls = caget(PVs(bpms, 'CK:MCL'), datatype = catools.dbr_time_long)
    print '  Restoring Normal mode'
    caput(PVs(evrs, 'TRIG:MODE'), 'Normal', catools.dbr_string)
    
    AnalyseTimes('Time', bpms, [mcl.dbr.stamp for mcl in mcls])
    

    
def CheckMachineClocks(lb_bpms, bs_bpms, sr_bpms, ts_bpms, evrs, trigger):
    print 'Checking machine clocks'
    print '  Disabling events'
    caput(PVs(evrs, 'TRIG:MODE'), 'Synchronised', catools.dbr_string)
    time.sleep(5)
    print '  Triggering'
    trigger()
    time.sleep(5)

    for location, bpms in (
            ('LB', lb_bpms), ('BS', bs_bpms), ('SR', sr_bpms),
            ('TS', ts_bpms)):
        if bpms:
            mcls = caget(PVs(bpms, 'CK:MCL'))
            mchs = caget(PVs(bpms, 'CK:MCH'))

            AnalyseTimes(location, bpms,
                [mcl.value + 2**31 * mch.value
                    for mcl, mch in zip(mcls, mchs)])
    
    print '  Restoring event processing'
    caput(PVs(evrs, 'TRIG:MODE'), 'Normal', catools.dbr_string)

    

def parseArgs():
    parser = optparse.OptionParser(
        usage = ''''Usage: %prog [flags]
Synchronise Libera BPM clocks

To synchronise machine clocks on all BPMs in the storage ring run
    %prog -Sm
To test machine clock synchronisation on the storage ring run
    %prog -Smt''')

    parser.add_option(
        '-t', dest = 'test_only', action = 'store_true', default = False,
        help = 'Test synchronisation only')
    parser.add_option(
        '-n', dest = 'no_test', action = 'store_true', default = False,
        help = 'Disable synchronisation testing')
    parser.add_option(
        '-s', dest = 'system', action = 'store_true', default = False,
        help = 'Synchronise or test system clocks')
    parser.add_option(
        '-m', dest = 'machine', action = 'store_true', default = False,
        help = 'Synchronise or test machine clocks')
    parser.add_option(
        '-S', dest = 'SR', action = 'store_true', default = False,
        help = 'Synchronise or test storage ring')
    parser.add_option(
        '-I', dest = 'LBS', action = 'store_true', default = False,
        help = 'Synchronise or test injector')
    parser.add_option(
        '-L', dest = 'TS', action = 'store_true', default = False,
        help = 'Synchronise or test lab BPMs')

    options, arglist = parser.parse_args()
    if arglist:
        parser.error('No arguments supported')

    if not options.SR and not options.LBS and not options.TS:
        parser.error('Specify a region of the machine to synchronise')
    if not options.system and not options.machine:
        parser.error('Specify a class of clock to synchronise')
    if options.TS and (options.SR or options.LBS):
        parser.error('Cannot operate on synchrotron and lab together')

    return options


def main():
    options = parseArgs()

    if options.TS:
        AllBpms = TS_BPMS
        AllEvrs = TS_EVRS
        SystemTrigger = LabSystemTrigger
        MachineTrigger = LabMachineTrigger()
    else:
        AllBpms = []
        AllEvrs = []
        Triggers = [BS_TRIG]
        if options.LBS:
            AllBpms.extend(LB_BPMS + BS_BPMS)
            AllEvrs.extend(LB_EVRS + BS_EVRS)
            Triggers.append(LB_TRIG)
        if options.SR:
            AllBpms.extend(SR_BPMS)
            AllEvrs.extend(SR_EVRS)
        SystemTrigger = GlobalSystemTrigger
        MachineTrigger = GlobalMachineTrigger(Triggers)

    ActiveBpms = GetActiveBpms(AllBpms)
    ActiveEvrs = GetActiveEvrs(AllEvrs)
    
    if not options.test_only:
        if options.system:
            SynchroniseSystemClocks(ActiveBpms, ActiveEvrs, SystemTrigger)
        if options.machine:
            SynchroniseMachineClocks(ActiveBpms, MachineTrigger)
    if not options.no_test:
        if options.system:
            CheckSystemClocks(ActiveBpms, ActiveEvrs, SystemTrigger)
        if options.machine:
            SetActiveBpms = set(ActiveBpms)
            CheckMachineClocks(
                SetActiveBpms & set(LB_BPMS),
                SetActiveBpms & set(BS_BPMS),
                SetActiveBpms & set(SR_BPMS),
                SetActiveBpms & set(TS_BPMS),
                ActiveEvrs, SystemTrigger)
    

if __name__ == '__main__':
    main()
    

