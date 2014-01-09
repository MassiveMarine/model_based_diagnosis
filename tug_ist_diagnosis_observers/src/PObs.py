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
import numpy
class Property_Observer(object):

		def __init__(self, argv):
			rospy.init_node('PObs', anonymous=True)
			self.pub = rospy.Publisher('/observations', Observations)
			self.obs_msg = Observations()
			self.param_node = rospy.get_param('~node', 'Node')
			self.param_property = rospy.get_param('~property', 'MEM')
			self.param_max = rospy.get_param('~max_val', 0.2)
			self.param_mismatch_th = rospy.get_param('~mismatch_thr', 5)
			self.param_ws = rospy.get_param('~ws', 10)
			self.circular_queu = [0 for i in xrange(self.param_ws)]
			self.mismatch_counter = 0
			self.args = argv
			self.node_exist = False
			self.sum = 0
			if self.param_node[0] == '/':
				self.node = self.node[1:len(self.node)]
         
		def start(self):
			try:
				print 'PObs is up and has started publishsing observations.......'
				a = subprocess.Popen("rosnode info " + self.param_node , shell=True,stdout=subprocess.PIPE)
				parts = shlex.split(a.communicate()[0])
				indx = parts.index("Pid:")
				pid = parts[indx+1]
				if self.param_property.lower() == 'cpu':
					p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
					self.out = p.communicate()[0]
					self.out1 = shlex.split(self.out)
					self.circular_queu.append(float(self.out1[8]))
				else :
					p = subprocess.Popen("pmap -x %s" %pid, shell=True,stdout=subprocess.PIPE)
					self.out = p.communicate()[0]
					self.out1 = shlex.split(self.out)
					self.circular_queu.append(float(self.out1[len(self.out1)-3]))
				self.node_exist = True
			except:
				print 'ERROR: A problem reported. Please check if the node is running!'
				exit()

			while not rospy.is_shutdown():
				try:
					if self.node_exist == False:
						a = subprocess.Popen("rosnode info " + self.param_node , shell=True,stdout=subprocess.PIPE)
						parts = shlex.split(a.communicate()[0])
						indx = parts.index("Pid:")
						pid = parts[indx+1]
					avg_val = 0
					if self.param_property.lower() == 'cpu':
						p = subprocess.Popen("top -b -n 1 | grep -i %s" %pid, shell=True,stdout=subprocess.PIPE)
						self.out = p.communicate()[0]
						self.out1 = shlex.split(self.out)
						self.circular_queu.pop(0)
						self.circular_queu.append(float(self.out1[8]))
						#avg_val = float(self.out1[8])
						avg_val = numpy.mean(self.circular_queu)
					else:
						p = subprocess.Popen("pmap -x %s" %pid, shell=True,stdout=subprocess.PIPE)
						self.out = p.communicate()[0]
						self.out1 = shlex.split(self.out)
						self.circular_queu.pop(0)
						self.circular_queu.append(float(self.out1[len(self.out1)-3]))
						#avg_val = float(self.out1[len(self.out1)-3])
						avg_val = numpy.mean(self.circular_queu)
					self.obs_msg.out_time = rospy.get_time()
					rospy.loginfo('CalVal'+self.param_property+self.param_node+'='+str(avg_val)+',maxV='+str(self.param_max))
					if (avg_val <= float(self.param_max)+8):
						if self.mismatch_counter > 0:
							self.mismatch_counter = self.mismatch_counter - 1	
					else:
						self.mismatch_counter = self.mismatch_counter + 1
						if self.mismatch_counter > 2*self.param_mismatch_th:
							self.mismatch_counter = self.param_mismatch_th + 1

					self.publish_output()
				except:
					self.node_exist = False
					rospy.loginfo('disappear('+self.param_node+','+self.param_property+')')
					pass

					time.sleep(0.5)

				
		def publish_output(self):
			obs_msg = []
			if self.mismatch_counter < self.param_mismatch_th:
				rospy.loginfo('ok('+self.param_node+','+self.param_property+')')
				obs_msg.append('ok('+self.param_node+','+self.param_property+')')
				self.obs_msg.obs=obs_msg
				self.pub.publish(self.obs_msg)
				#self.pub.publish(Observations(time.time(),obs_msg))
			else:
				rospy.loginfo('~ok('+self.param_node+','+self.param_property+')')
				#print self.circular_queu
				#rospy.loginfo(self.param_node+' obtV='+str(obtained_val)+'maxV'+str(self.param_max))
				obs_msg.append('~ok('+self.param_node+','+self.param_property+')')
				self.obs_msg.obs=obs_msg
				self.pub.publish(self.obs_msg)
				#self.pub.publish(Observations(time.time(),obs_msg))
			

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

