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

import roslib.message;roslib.load_manifest('tug_ist_diagnosis_generator')
import rospy
import sys
import xmlrpclib
import os
import subprocess
import time
import shlex
import thread
from math import sqrt
import time
class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.mapping = []
		self.dataMAT = [[]]
		self.dataFile = open('safdarFile', 'w')
		
	def start(self):
		self.extract_nodes_topics()

	def end(self):
		self.dataFile.close()

	def callback(self,data,topic):
		curr_t = time.time()
		self.dataFile.write(str(curr_t)+'\n')
		rospy.loginfo('type = '+str(data.header))
		

	def extract_nodes_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		for topic in topicList:
			if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg'):
				continue
			#self.mapping[9] = topic[1]
			print topic[1]
			msg_class = roslib.message.get_message_class(topic[1])
			rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		rospy.spin()		
        
if __name__ == '__main__':
      generator = Generator()
      generator.start()
      generator.end()
