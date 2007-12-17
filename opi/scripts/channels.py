#!/usr/bin/env /home/tools/bin/python2.4
# Build a config file for StripTool from EDM

import sys, os, subprocess
import tempfile
from pkg_resources import require
require("dls.ca2")
from dls.ca2 import catools


if len(sys.argv) != 2:
    print "usage: channels.py <bpm>"
    sys.exit(0);

    
BPM = sys.argv[1]
Channels = ['%s:SC:C%sMAG' % (BPM, channel) for channel in '1234']

# Read the current values of the channels
ChannelValues = [c.value for c in catools.caget(Channels, timeout=5)]

# Create a temporary configuration file.
config = tempfile.NamedTemporaryFile(prefix='%s.' % BPM)

# write the strip tool header
config.write('''StripConfig                   1.2
Strip.Time.Timespan           300
Strip.Time.NumSamples         65536
Strip.Time.SampleInterval     1
Strip.Time.RefreshInterval    1
Strip.Option.GraphLineWidth   0
''')

# Write our field specific bits.  We zoom in to +-1% of the initial value of
# each button.
window_size = 0.001
for curve, (ChannelName, ChannelValue) in \
        enumerate(zip(Channels, ChannelValues)):
    min_value = ChannelValue - window_size
    max_value = ChannelValue + window_size
    config.write('''Strip.Curve.%(curve)d.Name     %(ChannelName)s
Strip.Curve.%(curve)d.Units    mm
Strip.Curve.%(curve)d.Min      %(min_value)g
Strip.Curve.%(curve)d.Max      %(max_value)g
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
