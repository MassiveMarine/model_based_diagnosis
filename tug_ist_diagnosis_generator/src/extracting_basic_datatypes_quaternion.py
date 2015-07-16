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
from geometry_msgs.msg import Quaternion
import time
import tf
class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.fields = []
		self.topic = ''
		self.mapping = []
		self.dataMAT = [[]]
		self.dataFile = open('safdarFile', 'w')
		
	def start(self):
		self.extract_nodes_topics()

	def end(self):
		self.dataFile.close()
	
	def rec_call(self,data,topic):
		try:    
			if data.__class__ == Quaternion:
				rpy = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])
			st = data._slot_types
			for l1 in data.__slots__:
				l = getattr(data,l1)
				self.rec_call(l,topic)
			
		except:
			pass
				
	def callback(self,data,topic):
		try:
			curr_t = time.time()
			self.rec_call(data,topic)
		except AttributeError:
			e = sys.exc_info()[0]	
			print e			


	def get_fieldsRecursive(self,field,msg_class):
		self.fields.append(field)
		
		basic_data_types = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
		basic_NOT_data_types = ['bool','string','time','std_msgs/Header','string[','float32[','float64[','int8[','uint[','int16[','uint16[','int32[','uint32[','int64[','uint64[','byte']

		t = ''
		ln = len(msg_class)
		msg_class_arr=''
		if msg_class[ln-1] == ']':
			j = msg_class.find('[')
			t = msg_class[j+1:ln-1]
			msg_class_arr = msg_class
			msg_class = msg_class[0:j+1]

		if (msg_class in basic_data_types):
			print self.fields, msg_class
			self.fields.pop()
		else:
			if msg_class not in basic_NOT_data_types:

				if msg_class[len(msg_class)-1] == '[':
					msg_class = msg_class[0:len(msg_class)-1]
				msg_class = roslib.message.get_message_class(msg_class)
				fields = msg_class.__slots__
				x = 0
				for itm in msg_class._slot_types:
					self.get_fieldsRecursive(fields[x],itm)
					x = x + 1
				self.fields.pop()
			else:
				self.fields.pop()
					

	def get_fields(self,statusMessage):
		basic_data_types = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
		basic_NOT_data_types = ['bool','string','time','std_msgs/Header','float32[]','float64[]']
		l = len(statusMessage)
		for i in xrange(l):
			msg_class = roslib.message.get_message_class(statusMessage[i][1])
			self.topic = statusMessage[i][0]
			print '--------',self.topic,'(',i,')---------'
			self.get_fieldsRecursive(statusMessage[i][0],statusMessage[i][1])

	def extract_nodes_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		self.get_fields(topicList)
		for topic in topicList:
			if (topic[0] == '/rosout') | (topic[0] == '/rosout_agg'):
				continue
			msg_class = roslib.message.get_message_class(topic[1])
			rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		print 'call back started . '	
		rospy.spin()		
        
if __name__ == '__main__':
      generator = Generator()
      generator.start()
      generator.end()
