#!/usr/bin/env python2.4

from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools


import re
import sys
import os


pattern = re.compile(sys.argv[1])
pv = sys.argv[2]
value = sys.argv[3]

allbpms = os.path.join(os.path.dirname(__file__), 'allbpms')
bpms = [line[:-1] for line in file(allbpms) if pattern.search(line)]

sl = catools.caput(
    ['%s:%s' % (bpm, pv) for bpm in bpms], [value for bpm in bpms],
    datatype = catools.dbr_string)

errorcode = 0
for s, bpm in zip(sl, bpms):
    if s != catools.ECA_NORMAL:
        print >>sys.stderr, bpm, catools.ca_message(s)
        errorcode = 1

sys.exit(errorcode)
