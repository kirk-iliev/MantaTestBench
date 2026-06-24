#!/usr/bin/env python

from __future__ import print_function
import argparse
import epics
import sys
import time

parser = argparse.ArgumentParser(description='Fake facility timing system to exercise dual event generator.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-c', '--count', type=int, default=1, help='Number of cycles to request')
parser.add_argument('-e', '--evg', default='testEVG:', help='Event generator record name prefix')
parser.add_argument('-m', '--monitor', action='store_true', help='Monitor and display event sequences')
parser.add_argument('-t', '--test', default='test', help='Timing system test prefix')
parser.add_argument('-v', '--verbose', action='store_true', help='Show outgoing requests')
args = parser.parse_args()

# Connect to PV and verify connection
def pv(name):
    pv = epics.PV(name, connection_timeout=1.0)
    pv.get()
    if not pv.connect():
        print('Unable to connect to "%s"' % (name))
        sys.exit(1)
    return pv

# Synchronize with booster cycle
cycleDone = False
seqStatusWasBusy = None
def seqStatusCallback(pvname=None, value=None, **kws):
    global cycleDone, seqStatusWasBusy
    seqStatIsBusy = (value & 0x10) != 0
    if seqStatusWasBusy == None:
        seqStatusWasBusy = seqStatIsBusy
    if not seqStatIsBusy and seqStatusWasBusy:
        cycleDone = True
    seqStatusWasBusy = seqStatIsBusy
seqStatus = pv(args.evg + 'E1:seqStatus')
seqStatus.add_callback(seqStatusCallback)

# Injection request
TARGET_BUCKET         = 0
GUN_BUNCHES           = 1
INJ_MODE              = 2
GUN_INHIBIT           = 3
INJ_FIELD_SYNC_DELAY  = 4
EXTR_FIELD_SYNC_DELAY = 5
SEQUENCE              = 6
request = [1, 4, 40, 0, 1832886, 60239272, 1]
request[SEQUENCE] = int(time.time())
requestPV = pv(args.test + 'TimInjReq')
bucketIndex = 0

# Show the sequence requests
then = 0.0
def sequenceCallback(pvname=None, value=None, **kws):
    global then
    now = time.time()
    if then == 0:
        then = now
    print("+%.6f"%(now - then))
    then = now
    i = 0
    while True:
        gap = value[i]
        evCode = value[i+1]
        cat = value[i+2]
        i += 3
        print(f"{gap}:{evCode}:{cat}")
        if evCode == 127: break;

if args.monitor:
    # Show event generator updates
    sequence = pv(args.evg + 'E1:SEQ1')
    sequence.add_callback(sequenceCallback)

# Send requests until count limit has been reached
cycleDone = False
while args.count > 0:
    while not cycleDone:
        time.sleep(0.05)
    cycleDone = False
    bucketIndex = bucketIndex % 328
    request[TARGET_BUCKET] = bucketIndex + 1
    request[SEQUENCE] += 1
    requestPV.put(request)
    if (args.verbose): print(request)
    args.count -= 1
if args.monitor:
    time.sleep(1.0)
