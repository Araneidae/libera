#!/usr/bin/env python2.4

from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools

import re
import sys, os


allbpms = os.path.join(os.path.dirname(__file__), 'allbpms')

pattern = re.compile(sys.argv[1])
pv = sys.argv[2]


        

bpms = [line[:-1] for line in file(allbpms) if pattern.search(line)]
pvs = ['%s:%s' % (bpm, pv) for bpm in bpms]

errorcode = 0
for v in catools.caget(pvs, timeout = 5, throw = False):
    if v.status == catools.ECA_NORMAL:
        print v.name, v.value
    else:
        print >>sys.stderr, v.name, catools.ca_message(v.status)
        errorcode = 1

sys.exit(errorcode)
