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
from std_msgs.msg._Header import Header
from rosgraph_msgs.msg._Log import Log
import time
import tf
import signal

class ColumnsRPY(object):
	def __init__(self, name):
		self.name = name
		self.roll = []
		self.pitch = []
		self.yaw = []
	def set_RPY(self,roll,pitch,yaw):
		self.roll.append(roll)
		self.pitch.append(pitch)
		self.yaw.append(yaw)
	def get_roll(self):
		return self.roll
	def get_pitch(self):
		return self.pitch
	def get_yaw(self):
		return self.yaw


class Generator(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.fields = []
		self.topic = ''
		self.ok_topics_list = []
		self.topic_has_value = False
		self.Columns_list = []
		self.ColumnsRPY_list = []
		self.counter = 0
		self.mapping = []
		self.dataMAT = [[]]
		self.dataFile = open('safdarFile', 'w')
		self.basic_data_types = ['int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
		self.basic_NOT_data_types = ['bool','str','string','time','std_msgs/Header','string[','float32[','float64[','int8[','uint8[','int16[','uint16[','int32[','uint32[','int64[','uint64[','byte', 'duration']
		
	def start(self):
		#signal.signal(signal.SIGINT, self.signal_handler)
		self.extract_nodes_topics()
		#self.get_fields()

        def signal_handler(self,signum, frame):
		for col in self.ColumnsRPY_list:
			print col[0]

	def end(self):
		self.dataFile.close()

	def callback(self,data,topic):
		try:
			self.fields = []
			curr_t = time.time()
			self.rec_callB(data,[topic,topic])

		except AttributeError:
			self.fields = []
			e = sys.exc_info()[0]	
			print e			

	def rec_callB(self,data,field):
		try:   
			self.fields.append(field[1])
			if data.__class__ == Quaternion:
				#print topic, data.__class__
				list_name = ''
				l = len(self.fields)
				for f in self.fields:
					list_name = list_name + f + '_'
				rpy = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])
				for col in self.ColumnsRPY_list:
					if col[0] == list_name:
						col[1].set_RPY(rpy[0], rpy[1], rpy[2])
						break
				print l,list_name,'=', rpy[0], rpy[1], rpy[2]
				self.fields.pop()
				#self.fields.pop()
			else:
				if (type(data) == int) | (type(data) == float):
					list_name = ''
					l = len(self.fields)
					for f in self.fields:
						list_name = list_name + f + '_'
					#print list_name,'=', data
					self.fields.pop()
				else:
					if (type(data) != Header) & (type(data)!=bool) & (type(data)!=str) & (type(data) != Log) :
						#if type(data) == list:
						#	print data #, msg_cls
						#print type(data), data.__class__#,data.__slots__#, msg_cls
						#print data.__slots__
						#print data._slot_types
						for l1 in data.__slots__:
							#print l1
							l = getattr(data,l1)
							self.rec_callB(l,[field[0],l1])
						#print 'POPED3',self.fields.pop()
					else:
						#print data.__class__
						self.fields.pop()#
						a = 1
		except:
			self.fields = []
			pass
			#print sys.exc_info()[0]

	def rec_call(self,msg_class,field):
		#if type(msg_class) == str:
		#	t = ''
		#	ln = len(msg_class)
		#	msg_class_arr=''
		#	if msg_class[ln-1] == ']':
		#		j = msg_class.find('[')
		#		t = msg_class[j+1:ln-1]
		#		msg_class_arr = msg_class
		#		msg_class = msg_class[0:j+1]

		if (msg_class in self.basic_data_types):
			#print msg_class, ':', field[1], '=', getattr(field[0],field[1])
			a= 1
		else:
			if (msg_class not in self.basic_NOT_data_types):
				try:
					#msg_class = roslib.message.get_message_class(msg_class)
					fields = msg_class.__slots__
					types = msg_class._slot_types
					#print len(types)
					for i in xrange(len(types)):
						self.rec_call(types[i],[msg_class, fields[i]])
				except:
					#print msg_class, types
					pass
					#print 'ERROR',msg_class

		#print fields
		#fields = msg_class.__slots__
		#types = msg_class._slot_types
		#for i in xrange(len(types)):
		#	if (types[i] in self.basic_data_types):
				#print fields[i],'=',getattr(msg_class, fields[i]),types[i]
		#		a = 1
		#	else:
		#		print types[i]
			#self.rec_call(types[i],'')
			#print msg_class, '=', field
		 
	def get_fieldsRecursive(self,field,msg_class):
		self.fields.append(field)
		t = ''
		ln = len(msg_class)
		msg_class_arr=''
		if msg_class[ln-1] == ']':
			j = msg_class.find('[')
			t = msg_class[j+1:ln-1]
			msg_class_arr = msg_class
			msg_class = msg_class[0:j+1]
		if (msg_class in self.basic_data_types):
			list_name = ''
			for f in self.fields:
				list_name = list_name + f + '_'
			self.counter = self.counter + 1
			self.topic_has_value = True
			self.ok_topics_list.append(self.topic)
			print self.counter, ':', list_name, msg_class
			#self.Columns_list.append((list_name, Columns(list_name)))
			if list_name.find('orientation') > -1:
				self.ColumnsRPY_list.append((list_name, ColumnsRPY(list_name)))
			else:
				self.Columns_list.append((list_name, []))
			self.fields.pop()
		else:
			if msg_class not in self.basic_NOT_data_types:
				if msg_class[len(msg_class)-1] == '[':
					msg_class = msg_class[0:len(msg_class)-1]

				msg_class = roslib.message.get_message_class(msg_class)
				#print msg_class
				fields = msg_class.__slots__
				x = 0
				for itm in msg_class._slot_types:
					#print getattr(msg_class,fields[x])
					self.get_fieldsRecursive(fields[x],itm)
					x = x + 1
				self.fields.pop()
			else:
				self.fields.pop()

	def extract_nodes_topics(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		l = len(topicList)
		for i in xrange(l):
			print '--------',topicList[i][0],i,'---------'
			msg_class = roslib.message.get_message_class(topicList[i][1])
			#print msg_class._slot_types
			#print msg_class.__slots__
			self.topic = topicList[i][0]
			self.get_fieldsRecursive(topicList[i][0],topicList[i][1])
			#if self.topic_has_value == True:
			#	print self.topic,' ACCEPTED!!!'
			#	self.topic_has_value = False
			#else:
			#	print self.topic,' REJECTED!!!'

		for obj in self.Columns_list:
			print 'OBJ: name=',obj[0],' class:', obj[1]

		for obj in self.ColumnsRPY_list:
			print 'OBJRPY: name=',obj[0],' class:', obj[1]
		
		for topic in topicList:
			if (topic[0] in self.ok_topics_list) & (topic[0].find('imu')<0):
				msg_class = roslib.message.get_message_class(topic[1])
				rospy.Subscriber(topic[0], msg_class, self.callback, topic[0], 100)
		print 'call back started ...'
		rospy.spin()		
        
if __name__ == '__main__':
      generator = Generator()
      generator.start()
      generator.end()
