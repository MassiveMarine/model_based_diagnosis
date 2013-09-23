#!/usr/bin/env python

# kalman1.py
# written by Greg Czerniak (email is greg {aT] czerniak [dOt} info )
#
# Implements a single-variable linear Kalman filter.
#
# Note: This code is part of a larger tutorial "Kalman Filters for Undergrads"
# located at http://greg.czerniak.info/node/5.
import random
import numpy
import pylab
import sys
import xmlrpclib
import os
import subprocess
import time
import shlex
import thread
from math import sqrt
import signal
import rospy
try:
	rospy.init_node('GObs_Node', anonymous=True)
	node_pid = 10785
	p = subprocess.Popen("ps -p 10785 -o %cpu,%mem",shell=True,stdout=subprocess.PIPE)
	out = p.communicate()[0]
	#print out
	out1 = shlex.split(out)
	print out1
	#print int(out1[len(out1)-3]), float(int(out1[len(out1)-3]))/1000
	#print rospy.get_time(), '---', time.time()

except:
	e = sys.exc_info()[0]
  	print e
	print 'Program terminated....'
