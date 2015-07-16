#!/usr/bin/env python

##
# The Diagnostic Observer observes device status on the /diagnostics topic whether its OK, WARNING or ERROR.
# and provides /Diagnosic_Observation topic compatible for our Model Based Diagnosis.
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

import roslib.message; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np

class testNode(object):

    def __init__(self):
	rospy.init_node('testNode', anonymous=False)
	self.pub = rospy.Publisher('testNode_topic', String)
         
    def start(self):
	#rospy.Subscriber("scan", LaserScan, self.access_dataA)
	#rospy.Subscriber("laser_node_topic", String, self.access_dataL)
	#rospy.spin
	x = np.sqrt(0.651666667)
	print "\nx =",x
    def access_dataA(self,data):
	print data
	type1 = data._connection_header['topic']
	print type1

    def access_dataL(self,data):
	self.pub.publish(data)

if __name__ == '__main__':
	tn = testNode()
	tn.start()
