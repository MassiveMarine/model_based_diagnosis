#!/usr/bin/env python

##
# GObs.py is a General Observer to observer a topic and provide observation on /observations topic.
# It needs three parameters Required frequency, frequency deviation and window size.
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
import sys
import xmlrpclib
import os
from std_msgs.msg import String
from tug_ist_diagnosis_msgs.msg import Observations
import time
from array import array
import thread
import re
import traceback
import numpy

class General_Observer(object):

    def __init__(self):
				rospy.init_node('GObs_Node', anonymous=True)
				self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
				self.caller_id = '/script'
				self.obs_msg = []
				self.topic_type = ""
				self.topic_name = ""
				self.msg = ""
				#t = rospy.get_rostime()
				self.prev_t = 0
				self.curr_t = 100000
				self.mid_t = 0
				self.mid_ptr = -1
				self.mean = 0
				self.delta_mean = 0
				self.pub = rospy.Publisher('/observations', Observations)
				self.param_topic = rospy.get_param('~topic', '/Topic')
				self.param_frq =  rospy.get_param('~frq', 10)
				self.param_dev = rospy.get_param('~dev', 1)
				self.param_ws = rospy.get_param('~ws', 10)
				self.param_mismatch_th = rospy.get_param('~mismatch_th', 5)
				self.circular_queu_t = []
				self.delta_t_queu = []
				#self.delta_t = [0 for i in xrange(self.param_ws)]
				self.sum = 1
				self.count = 0
				self.last_occ_t = 0
				self.mismatch_th_counter = 0;
				if self.param_topic[0] == '/':
					self.param_topic = self.param_topic[1:len(self.param_topic)]
				thread.start_new_thread(self.check_topic,(self.param_topic,1))
				#thread.start_new_thread(self.make_output,(self.param_topic,0.5))
				
				        
    def start(self):         
				if self.param_topic[0] != '/':
					self.param_topic = "/%s" % (self.param_topic)
				self.topic_name = self.param_topic[1:len(self.param_topic)]
				#print self.param_topic, self.param_frq, self.param_dev, self.param_ws
				topic_found = False
				firstcheck = True
				while firstcheck:
					pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
					for item in topicList:
						if item[0] == self.param_topic:
							self.param_topic = item[0]
							self.topic_type = item[1]
							topic_found = True
					if topic_found == True:
						firstcheck = False
						msg_class = roslib.message.get_message_class(self.topic_type)
						rospy.Subscriber(self.param_topic, msg_class, self.callback)
						#t = rospy.get_rostime()
						#self.prev_t = t.secs + t.nsecs*1e-9
						self.prev_t = time.time()
						rospy.spin()
					else:
						self.msg = '~ok('+self.topic_name+')'
						self.pub.publish(Observations(time.time(),[self.msg]))
					time.sleep(1)
				t = rospy.get_rostime()
				self.mid_t = t.secs + t.nsecs*1e-9
				print t.secs, 'MT:',self.mid_t
				
					

    def callback(self,data):
	#self.curr_t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
	self.curr_t = time.time()	
	#self.circular_queu_t.append(self.curr_t)
	if (self.mid_t-(self.param_ws/2)) < self.curr_t and self.curr_t < (self.mid_t+(self.param_ws/2)):
		#print 'NO:',len(self.circular_queu_t),'mid_t',self.mid_t,'::',self.mid_t-self.param_ws,' <', self.curr_t, '<', self.mid_t+self.param_ws
		self.circular_queu_t.append(self.curr_t)
		self.delta_t_queu.append(self.curr_t-self.prev_t)
	else:
		#print 'Length:', len(self.circular_queu_t)
		#self.delta_mean = self.calculate_delta()
		self.make_output()
		self.circular_queu_t.append(self.curr_t)
		self.delta_t_queu.append(self.curr_t-self.prev_t)
		self.mid_ptr = self.mid_ptr + 1
		self.mid_t = self.circular_queu_t[self.mid_ptr]
		#print 'MT:', self.mid_t,'NO:',len(self.circular_queu_t),'mid_t',self.mid_t,'::',(self.mid_t-self.param_ws),' <', self.curr_t, '<', (self.mid_t+self.param_ws), self.circular_queu_t[0]
		while self.circular_queu_t[0] < (self.mid_t-self.param_ws):
			self.circular_queu_t.pop(0)
			self.mid_ptr = self.mid_ptr - 1
			self.delta_t_queu.pop(0)
			#print 'POPPPPPPPPPPPPPPPPED'
	#self.curr_t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
	#self.circular_queu_t.append(self.curr_t)
	self.prev_t = self.curr_t
	self.last_occ_t = time.time()
		
						
    def calculate_delta(self):
	#sm = 0
	#n = 0
	dList = []
	for i in range(1,len(self.circular_queu_t)):
		dList.append(self.circular_queu_t[i] - self.circular_queu_t[i-1])
	#avg = sm/n
	#print n, avg, self.param_frq
	m = numpy.mean(dList)
	return m


    def make_output(self):
		#time.sleep(delay)
		obs_msg = []
		delta_mean = numpy.mean(self.delta_t_queu)
		print 'DeltaQ length:',len(self.delta_t_queu)
		rospy.loginfo('MisM:'+str(self.mismatch_th_counter)+','+str(self.param_frq-self.param_dev)+'CalcDelta='+str(delta_mean)+','+str(self.param_frq+self.param_dev))
		print 'Req LowLimit:',1/(self.param_frq+self.param_dev),'Cal_Frq:',1/delta_mean,'Req HighLimit:',1/(self.param_frq-self.param_dev)
		if (self.param_frq-self.param_dev <= delta_mean) and (delta_mean <= self.param_frq+self.param_dev):
			if self.mismatch_th_counter > 0:
				self.mismatch_th_counter = self.mismatch_th_counter-1
			#self.msg = 'ok('+self.topic_name+')'
			#rospy.loginfo('ok('+self.topic_name+')')
			#obs_msg.append(self.msg)
			#self.pub.publish(Observations(time.time(),obs_msg))
		else:
			self.mismatch_th_counter = self.mismatch_th_counter + 1
			if self.mismatch_th_counter > 2*self.param_mismatch_th:
				self.mismatch_th_counter = self.param_mismatch_th + 1
		if self.mismatch_th_counter > self.param_mismatch_th:
			self.msg = '~ok('+self.topic_name+')'
			rospy.loginfo('~ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))
		else:
			self.msg = 'ok('+self.topic_name+')'
			rospy.loginfo('ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))

    def make_output1(self,diff_freq,delay,*args):
	#rospy.loginfo('MEAN--'+self.delta_mean)
	while not rospy.is_shutdown():
		time.sleep(delay)
		obs_msg = []
		print self.mismatch_th_counter,self.param_frq-self.param_dev, self.delta_mean, self.param_frq+self.param_dev
		print 'Req LowLimit:',1/self.param_frq,'Cal_Frq:',1/self.delta_mean
		if (self.param_frq-self.param_dev <= self.delta_mean) and (self.delta_mean <= self.param_frq+self.param_dev):
			if self.mismatch_th_counter > 0:
				self.mismatch_th_counter = self.mismatch_th_counter-1
			#self.msg = 'ok('+self.topic_name+')'
			#rospy.loginfo('ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))
		else:
			self.mismatch_th_counter = self.mismatch_th_counter + 1
			if self.mismatch_th_counter > 2*self.param_mismatch_th:
				self.mismatch_th_counter = self.param_mismatch_th + 1
		if self.mismatch_th_counter > self.param_mismatch_th:
			self.msg = '~ok('+self.topic_name+')'
			rospy.loginfo('~ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))
		else:
			self.msg = 'ok('+self.topic_name+')'
			rospy.loginfo('ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))

    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s/float(self.param_ws)

    def check_topic(self,string,sleeptime,*args):
				try:
					while not rospy.is_shutdown():
						t = 0
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
						#print topicList
						for item in topicList:
							#print item[0][1:],string
							if item[0][1:] == string:
									t = 1
									if time.time() - self.last_occ_t > 3:
										rospy.loginfo('~ok('+self.topic_name+')1from Thread')
										self.pub.publish(Observations(time.time(),['~ok('+self.topic_name+')']))
									break
						if t == 0:
								t = 1
								self.msg = '~ok('+self.topic_name+')'
								rospy.loginfo('~ok('+self.topic_name+')2 from Thread')
								self.pub.publish(Observations(time.time(),[self.msg]))
						time.sleep(sleeptime) #sleep for a specified amount of time.
				except:
						 	print "An unhandled exception occured, here's the traceback!"
							traceback.print_exc()


    def throws():
								raise RuntimeError('this is the error message')
												
    def report_error(self):
				print '\nrosrun tug_ist_diagnosis_observers GObs.py <Topic_name> <Frequency> <FreqDeviation> <WindowSize>'
				print 'e.g rosrun tug_ist_diagnosis_observers GObs.py _topic:=scan _frq:=10 _dev:=1 _ws:=10'
				sys.exit(os.EX_USAGE)
        
if __name__ == '__main__':
      GObs = General_Observer()
      GObs.start()
      
