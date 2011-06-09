#!/usr/bin/env python2.4

# Simple SNTP client.

import sys
import socket
import struct
import time


# Want to write
#   time.mktime((1900, 1, 1, 0, 0, 0, 0, 0, 0)) - time.mktime(time.gmtime(0)))
# but unfortunately this overflows.
ntp_epoch = (365 * 70 + 17) * 24 * 3600

def print_ntp_date(timestamp):
    if timestamp:
        seconds = int(timestamp >> 32) - ntp_epoch
        fraction = float(timestamp & 0xffffffff) / 2**32
        return '%s.%06d' % (
            time.strftime('%Y/%m/%d %H:%M:%S', time.gmtime(seconds)),
            int(fraction * 1e6))
    else:
        return 'no timestamp'


request_packet = ''.join(map(chr, [(3 << 3) | 3] + 47 * [0]))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sock.settimeout(2)
sock.connect((sys.argv[1], 123))
sock.send(request_packet)
response = sock.recv(256)
sock.close()

li_vn_mode, stratum, ppoll, precision, \
    rootdelay, rootdispersion,  \
    reftime, org, rec, xmt = struct.unpack('!BBbbii4xQQQQ', response)

li = (li_vn_mode >> 6) & 3
vn = (li_vn_mode >> 3) & 7
mode = li_vn_mode & 7
assert mode == 4, 'Unexpected response'


if stratum == 0:
    refid = repr(response[12:16])
else:
    refid = '%d.%d.%d.%d' % struct.unpack('BBBB', response[12:16])

li_lookup = [
    'normal', 'extra second', 'missing second', 'not synchronised']
li_string = li_lookup[li]

ppoll_s = 2 ** float(ppoll)
precision_us = 1e6 * 2 ** float(precision)
rootdelay_ms = 1e3 * float(rootdelay) / 2**16
rootdispersion_ms = 1e3 * float(rootdispersion) / 2**16
reftime_ts = print_ntp_date(reftime)
org_ts = print_ntp_date(org)
rec_ts = print_ntp_date(rec)
xmt_ts = print_ntp_date(xmt)

print '''Leap indicator = %(li)d (%(li_string)s), SNTP version = %(vn)d
Stratum = %(stratum)d, refid = %(refid)s
poll interval = %(ppoll)d = %(ppoll_s)g seconds
precision = %(precision)d = %(precision_us)g microseconds
rootdelay = %(rootdelay)d = %(rootdelay_ms)g milliseconds
rootdispersion = %(rootdispersion)d = %(rootdispersion_ms)g milliseconds
timestamps:
    reference = %(reftime_ts)s
    originate = %(org_ts)s
    receive   = %(rec_ts)s
    transmit  = %(xmt_ts)s''' % locals()

print 'Raw response:'
for i in range(0, 48, 16):
    print '    %s' % ' '.join('%02x' % ord(c) for c in response[i:i+16])
