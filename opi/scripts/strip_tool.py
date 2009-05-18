#!/usr/bin/env /home/tools/bin/python2.4
# Build a config file for StripTool from EDM

import sys, os, subprocess
import tempfile
from pkg_resources import require
require('cothread')
from cothread import catools


try:
    pattern, channels, window_size, interval, BPM = sys.argv[1:]
    window_size = float(window_size)
    interval = float(interval)
except:
    print \
'''usage: strip_tool.py <pattern> <channels> <window> <interval> <bpm>

Examples:

    strip_tool.py SA:%s     ABCD 0.01  0.1 <bpm>
    strip_tool.py SC:C%sMAG 1234 0.001 1   <bpm>
'''
    sys.exit(0);

    
Channels = ['%s:%s' % (BPM, pattern % channel) for channel in channels]

# Read the current values of the channels
ChannelValues = catools.caget(Channels, timeout=5)

# Create a temporary configuration file.
config = tempfile.NamedTemporaryFile(prefix='%s.' % BPM)

# write the strip tool header
config.write('''StripConfig                   1.2
Strip.Time.Timespan           300
Strip.Time.NumSamples         65536
Strip.Time.SampleInterval     %(interval)g
Strip.Time.RefreshInterval    %(interval)g
Strip.Option.GraphLineWidth   0
''' % locals())

# Write our field specific bits.  We zoom in to +-1% of the initial value of
# each button.
for curve, (ChannelName, ChannelValue) in \
        enumerate(zip(Channels, ChannelValues)):
    min_value = ChannelValue * (1 - window_size)
    max_value = ChannelValue * (1 + window_size)
    config.write('''Strip.Curve.%(curve)d.Name     %(ChannelName)s
Strip.Curve.%(curve)d.Units    mm
Strip.Curve.%(curve)d.Min      %(min_value)g
Strip.Curve.%(curve)d.Max      %(max_value)g
Strip.Curve.%(curve)d.Comment  Dummy Comment
''' % locals())
config.flush()

# run StripTool with our config file
try:
    retcode = subprocess.call('StripTool ' + config.name, shell=True)
    if retcode < 0:
        print >>sys.stderr, 'StripTool was terminated by signal', -retcode
    else:
        print >>sys.stderr, 'StripTool returned', retcode
except OSError, e:
    print >>sys.stderr, 'Execution failed:', e
    
# remove the temporary file
config.close()
