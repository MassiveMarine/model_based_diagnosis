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
		self.ok_topics_list = []
		self.topic_has_value = False
		self.counter = 0
		self.mapping = []
		self.dataMAT = [[]]
		self.dataFile = open('safdarFile', 'w')
		
	def start(self):
		self.extract_nodes_topics()
		#self.get_fields()

	def end(self):
		self.dataFile.close()

	#def rec_call(self,data,topic):
	#	try:    
	#		if data.__class__ == Quaternion:
				#print topic, data.__class__
	#			rpy = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])
				#rpy = tf.transformations.euler_from_quaternion([eval(data.x),eval(data.y),eval(data.z),eval(data.w)])
				#print topic,rpy[0], rpy[1], rpy[2]
			#else:
			#	print topic,data.__class__
	#		st = data._slot_types
			#print st,len(st)
	#		for l1 in data.__slots__:
	#			l = getattr(data,l1)
	#			self.rec_call(l,topic)
				#print '+++++',l1
			#print st
			#	if (c.find('Header')<0) and (c.find('string')<0) and (c.find('int')<0) and (c.find('float')<0):
			#		l = getattr(st,c)
			
	#	except:
	#		pass
				
	def callback(self,data,topic):
		try:
			curr_t = time.time()
			#if topic == '/pose':
			#print '--------------'
			self.rec_call(data,topic)
			#self.dataFile.write(str(curr_t)+'\n')
			#for l1 in data.__slots__:
			#	print l1,type(l1), l1.__slots__
			#str1 = type(data)
			#print type(data)
			#print 'TOPIC = ',topic,': types:',data._slot_types
			#if topic == '/pose':
			#	print '+++++++++++++++'
			#	for l1 in data.__slots__:
			#		l = getattr(data,l1)
			#		try:
			#			print l1,l._slot_types
			#			for l3 in l.__slots__:
			#				m = getattr(l,l3)
			#				try:
			#					print l3
			#					#for l4 in m.__slots__:
			#					#	o = getattr(m,l4)
			#					#try:
			#					#	print l4,o.__slots__
			#					#except: 
			#					#	pass
			#				except:
			#					pass
			#		except:
			#		 	pass
				#print '------------------'
				#for l1 in data._slot_types:
					#print l1				
				#for l1 in data.__slots__:
				#	l = getattr(data,l1)
				#	print l._slot_types,str(l).find('string')
					#l = getattr(data,l1)
					#print l1,l._slot_types
					#print l1._slot_types, l1.__slots__
			#	for l2 in l1._slot_types:
			#		print l2
			#print '------------\n'
			#for l1 in data.__slots__:
			#	print l1,type(l1),l1.__slot__types,'#####'
			#	#print l1.__slots__
			#	l = getattr(data,l1)
			#	for l2 in l.__slots__:
			#		print l2,type(l2)
			#msg_class = roslib.message.get_message_class(topic)
			#print msg_class
			#rospy.loginfo('type = '+str(data))
		except AttributeError:
			e = sys.exc_info()[0]	
			print e			
			#pass

	def rec_call(self,data,field):
		self.ok_topics_list.append(field[0])
		self.topic = field[1] + '_'
		try:    
			if data.__class__ == Quaternion:
				#print topic, data.__class__
				rpy = tf.transformations.euler_from_quaternion([data.x,data.y,data.z,data.w])
			else:
				if (type(data) == int) | (type(data) == float):
					print field[0],'_',self.topic,'==',data,'\n\n--------------------'
					self.topic = self.ok_topics_list.pop(0) + '_'
					self.ok_topics_list.clear()
				else:
					st = data._slot_types
					for l1 in data.__slots__:
						print l1
						l = getattr(data,l1)
						self.rec_call(l,[field[0],l1])
		except:
			#pass
			print sys.exc_info()[0]

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
			list_name = ''
			for f in self.fields:
				list_name = list_name + f + '_'
			self.counter = self.counter + 1
			self.topic_has_value = True
			self.ok_topics_list.append(self.topic)
			print self.counter, ':', list_name[0:len(list_name)-1], msg_class
			self.fields.pop()
		else:
			if msg_class not in basic_NOT_data_types:
				if msg_class[len(msg_class)-1] == '[':
					msg_class = msg_class[0:len(msg_class)-1]
				msg_class = roslib.message.get_message_class(msg_class)
				print msg_class
				fields = msg_class.__slots__
				x = 0
				for itm in msg_class._slot_types:
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
		for topic in topicList:
			if (topic[0] in self.ok_topics_list) & (topic[0].find('imu')<0):
				msg_class = roslib.message.get_message_class(topic[1])
				rospy.Subscriber(topic[0], msg_class, self.callback, [topic[0], ''], 100)
		print 'call back started ...'
		rospy.spin()		
        
if __name__ == '__main__':
      generator = Generator()
      generator.start()
      generator.end()
