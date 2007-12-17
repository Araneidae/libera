#!/usr/bin/env /home/tools/bin/python2.4
# Build a config file for StripTool from EDM

import sys, os, subprocess
import tempfile
from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools


if len(sys.argv) != 2:
    print "usage: buttons.py <bpm>"
    sys.exit(0);

    
BPM = sys.argv[1]
Buttons = ['%s:SA:%s' % (BPM, button) for button in 'ABCD']

# Read the current position of the buttons
ButtonValues = [b.value for b in catools.caget(Buttons, timeout=5)]

# Create a temporary configuration file.
config = tempfile.NamedTemporaryFile(prefix='%s.' % BPM)

# write the strip tool header
config.write('''StripConfig                   1.2
Strip.Time.Timespan           300
Strip.Time.NumSamples         65536
Strip.Time.SampleInterval     0.1
Strip.Time.RefreshInterval    0.1
Strip.Option.GraphLineWidth   0
''')

# Write our field specific bits.  We zoom in to +-1% of the initial value of
# each button.
window_size = 0.01 * sum(ButtonValues)/4.0
for curve, (ButtonName, ButtonValue) in enumerate(zip(Buttons, ButtonValues)):
    min_value = ButtonValue - window_size
    max_value = ButtonValue + window_size
    config.write('''Strip.Curve.%(curve)d.Name     %(ButtonName)s
Strip.Curve.%(curve)d.Units    mm
Strip.Curve.%(curve)d.Min      %(min_value)d
Strip.Curve.%(curve)d.Max      %(max_value)d
Strip.Curve.%(curve)d.Comment  Dummy Comment
''' % locals())
config.flush()

# run StripTool with our config file
try:
    retcode = subprocess.call("StripTool " + config.name, shell=True)
    if retcode < 0:
        print >>sys.stderr, "StripTool was terminated by signal", -retcode
    else:
        print >>sys.stderr, "StripTool returned", retcode
except OSError, e:
    print >>sys.stderr, "Execution failed:", e
    
# remove the temporary file
config.close()
