#!/usr/bin/env python

##
# PObs.py is a Property Observer.
# It needs five parameters node name, property(either cpu or mem), maximum value, mismatch threshold and window size.
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

# The Property Observer observers a specific hardware/software property related to a node. 
# A property could be CPU usage , Memory usage or any other resourse.
# It publishes this property over /observations topic compatible for our Model Based Diagnosis.


import roslib; roslib.load_manifest('tug_ist_diagnosis_observers')
import commands
import rospy
import subprocess
import sys
import os
import shlex
from tug_ist_diagnosis_msgs.msg import Observations
import time

class Property_Observer(object):

		def __init__(self, argv):
			rospy.init_node('PObs', anonymous=True)
			self.pub = rospy.Publisher('/observations', Observations)
			self.param_node = rospy.get_param('~node', 'Node')
			self.param_property = rospy.get_param('~property', 'MEM')
			self.param_max = rospy.get_param('~max_val', 0.2)
			self.param_mis_th = rospy.get_param('~mismatch_thr', 5)
			self.param_ws = rospy.get_param('~ws', 10)
			self.circular_queu = [0 for i in xrange(self.param_ws)]
			self.mismatch_counter = 0
			self.args = argv
			self.sum = 0
			if self.param_node[0] == '/':
				self.node = self.node[1:len(self.node)]
         
		def start(self):
			print 'PObs is up and has started publishsing observations.......'
			a = subprocess.Popen("rosnode info " + self.param_node , shell=True,stdout=subprocess.PIPE)
			parts = shlex.split(a.communicate()[0])
			indx = parts.index("Pid:")
			pid = parts[indx+1]
			p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
			self.out = p.communicate()[0]
			self.out1 = shlex.split(self.out)
			if (self.param_property == 'CPU') | (self.param_property == 'cpu') :
				indx = 8
			else:
				indx = 9
			while not rospy.is_shutdown():
				self.circular_queu.pop(0)
				p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
				self.out = p.communicate()[0]
				print self.out
				self.out1 = shlex.split(self.out)
				self.circular_queu.append(float(self.out1[indx]))
				avg_val = self.average()
				self.publish_output(avg_val)

		def average(self):
			s = 0
			for val in self.circular_queu:
				print val
				s = s + val
			print "AVG=", s/self.param_ws
			return s/self.param_ws

			
					
		def publish_output(self,obtained_val):
			obs_msg = []
			if (obtained_val <= float(self.param_max)):
				if self.mismatch_counter != 0:
					self.mismatch_counter = self.mismatch_counter - 1	
				print 'ok('+self.param_node+','+self.param_property+')'
				obs_msg.append('ok('+self.param_node+','+self.param_property+')')
				self.pub.publish(Observations(time.time(),obs_msg))
			else:
				self.mismatch_counter = self.mismatch_counter + 1
				if self.mismatch_counter < self.param_mis_th:
					print 'ok('+self.param_node+','+self.param_property+')'
					obs_msg.append('ok('+self.param_node+','+self.param_property+')')
					self.pub.publish(Observations(time.time(),obs_msg))
				else:
					print '~ok('+self.param_node+','+self.param_property+')'
					obs_msg.append('~ok('+self.param_node+','+self.param_property+')')
					self.pub.publish(Observations(time.time(),obs_msg))
			

def report_error():
	print """
rosrun tug_ist_diagnosis_observers PObs.py _node:=<Node_name> _property:=<Mem/Cpu> _max_value:=<maximumLimit> _mismatch_thr:=<mistmachthr> _ws:=<windowsize>
e.g rosrun tug_ist_diagnosis_observers PObs.py _node:=openni_camera _property:=CPU _max_value:=0.2 _mismatch_thr:=5'
"""
	sys.exit(os.EX_USAGE)        
    
if __name__ == '__main__':
			print len(sys.argv)
			if len(sys.argv) < 2: 
				report_error()
			pObs = Property_Observer(sys.argv)
			pObs.start()

