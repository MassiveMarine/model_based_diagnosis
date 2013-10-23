#!/usr/bin/env python

##
# observers_generator.py is a node that finds the nodes and topics and their related information in the running system.
# After finding necessary information it creates launch files in tug_ist_diagnosis_launch package and executes them automatically. 
# Copyright (c).2012. OWNER: Institute for Software Technology, TU Graz Austria.
# Authors: Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
# All rights reserved.
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

import roslib.message; roslib.load_manifest('tug_ist_diagnosis_generator')
import rospy
import sys
import xmlrpclib
import os
import subprocess
import time
import shlex
import thread
from math import sqrt
#import numpy
import numpy
import matplotlib.pyplot as plt

node = '/laser_node'
a = subprocess.Popen("rosnode info " + node , shell=True,stdout=subprocess.PIPE)
parts = shlex.split(a.communicate()[0])
indx = parts.index("Pid:")
pid = parts[indx+1]

print pid
									

def autocorr(x):
    result = numpy.correlate(x, x, mode='full')
    return result[result.size/2:]

def make_func(value_to_print):
    def _function():
        print value_to_print
    return _function

#f1 = make_func(1)
#f1()
#f2 = make_func(2)
#f2()


#print autocorr([1, 0, 1, 0, 1, 0, 1, 0])
#print autocorr([1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0])
#x = [1, 0, 1, 0, 1, 0, 1, 0]
#s1 = [1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0]
#s2 = [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1]
#s2 = [0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1]
#m1 = numpy.mean(s1)
#m2 = numpy.mean(s2)
#s11 = s1-m1
#s22 = s2-m2
#r1 = numpy.correlate(s11, s11, mode='full')
#r2 = numpy.correlate(s22, s22, mode='full')
#r1 = r1 * len(s1)
#r2 = r2 * len(s2)


#signal = NP.array([3,3,3,3,3,3,3,3,3,0,0,0,0,0,0,0,0,0,0,7,7,7,7,7,4,4,1,1,1,1,1,1,1])
#signal = NP.array([1,0,1,0,1,0,1,0,1,0,1,0,1])
#px, = NP.where(NP.ediff1d(signal) != 0)
#px = NP.r_[(0, px+1, [len(signal)])]
# collect the run-lengths for each unique item in the signal
#rx = [ (m, n, signal[m]) for (m, n) in zip(px[:-1], px[1:]) ]

#print r1
#print r2
#plt.plot(r2)
#plt.show()
#s1 = [1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0]
#s2 = [1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0]
#s3 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#s1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#print numpy.correlate(s1, s3)


