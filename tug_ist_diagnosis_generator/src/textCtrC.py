#!/usr/bin/env python
#import zmq
import signal

interrupted = False

def signal_handler(signum, frame):
	global interrupted
	print 'Ctrl Pressed!'
	interrupted = True
counter = 0
f = 1/float(2)
print "%.4f" % f
signal.signal(signal.SIGINT, signal_handler)
while True:
#	print 'hello'
	counter += 1
	if interrupted:
		print 'hello'
		break
