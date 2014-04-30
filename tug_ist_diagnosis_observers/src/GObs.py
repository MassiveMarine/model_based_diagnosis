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
				self.lock=thread.allocate_lock()
				self.msg = ""
<<<<<<< HEAD
				self.prev_t = 0
=======
				#t = rospy.get_rostime()
				self.prev_t = 0
				self.curr_t = 100000
				self.mid_t = 0
				self.mid_ptr = -1
				self.mean = 0
				self.delta_mean = 0
>>>>>>> safdar
				self.pub = rospy.Publisher('/observations', Observations)
				self.param_topic = rospy.get_param('~topic', '/Topic')
				self.param_frq =  rospy.get_param('~frq', 10)
				self.param_dev = rospy.get_param('~dev', 1)
				self.param_ws = rospy.get_param('~ws', 10)
				self.param_mismatch_th = rospy.get_param('~mismatch_th', 5)
<<<<<<< HEAD
				self.circular_queu = [0.1 for i in xrange(10)]
				#self.delta_t = [0 for i in xrange(self.param_ws)]
				self.delta_t_list = []
				self.curr_t_list = []
				self.delta_Ubound = self.param_frq +  1*self.param_dev 
				self.delta_Lbound = self.param_frq -  1*self.param_dev 
				self.output = False
				self.limit_t = 0
				self.sum = 1
				self.count = 0
				self.incall = True
				self.mismatch_th_counter = 0;
				if self.param_topic[0] == '/':
					self.param_topic = self.param_topic[1:len(self.param_topic)]
				#thread.start_new_thread(self.make_output,(self.param_topic,0.5))
        
        
=======
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
				
				        
>>>>>>> safdar
    def start(self):         
				#if self.param_topic[0] != '/':
				#	self.param_topic = "/%s" % (self.param_topic)
				#self.topic_name = self.param_topic[1:len(self.param_topic)]
				#print self.param_topic, self.param_frq, self.param_dev, self.param_ws
				topic_found = False
				firstcheck = True
				while firstcheck:
					pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
					for item in topicList:
						if item[0][1:] == self.param_topic:
							self.topic_name = item[0][1:]
							self.topic_type = item[1]
							topic_found = True
					if topic_found == True:
						firstcheck = False
						msg_class = roslib.message.get_message_class(self.topic_type)
						self.limit_t = time.time() + self.param_ws
						rospy.Subscriber(self.param_topic, msg_class, self.callback)
<<<<<<< HEAD
						self.prev_t = time.time()
						thread.start_new_thread(self.check_topic,(self.param_topic,0.1))
=======
						#t = rospy.get_rostime()
						#self.prev_t = t.secs + t.nsecs*1e-9
						self.prev_t = time.time()
>>>>>>> safdar
						rospy.spin()
					else:
						rospy.loginfo('~ok('+self.topic_name+')')
						self.msg = '~ok('+self.topic_name+')'
						self.pub.publish(Observations(time.time(),[self.msg]))
					time.sleep(1)
				t = rospy.get_rostime()
				self.mid_t = t.secs + t.nsecs*1e-9
				print t.secs, 'MT:',self.mid_t
				
					

    def callback(self,data):
<<<<<<< HEAD
	##self.lock.acquire()
	curr_t = time.time()
	self.curr_t_list.append(curr_t)
	self.incall = True
	delta_t = curr_t - self.prev_t
	self.delta_t_list.append(delta_t)
	self.prev_t = curr_t
	#rospy.loginfo(str(curr_t)+','+str(self.limit_t))
	if curr_t > self.limit_t:
		rospy.loginfo('CROSSED'+str(curr_t)+','+str(self.limit_t))
		occ = self.curr_t_list.pop(0)
		self.limit_t = occ + self.param_ws
		mean = numpy.mean(self.delta_t_list)
		self.publish_observation(mean)
		self.delta_t_list.pop(0)
	print 'SIZEEEEEEE',len(self.delta_t_list)
	##self.lock.release()
	self.incall = False
	##print time.time()-curr_t
		
    def publish_observation(self,mean):
		obs_msg = []
		rospy.loginfo('Obtainedmean'+str(mean)+' Ubounds='+str(self.delta_Ubound)+'Lbounds='+str(self.delta_Lbound))
		if (mean < self.delta_Ubound) & (mean > self.delta_Lbound):
			#rospy.loginfo('mean IN bounds')
			if self.mismatch_th_counter > 0:
				self.mismatch_th_counter = self.mismatch_th_counter -1
			self.msg = 'ok('+self.topic_name+')'
			rospy.loginfo('ok('+self.topic_name+')')
			obs_msg.append(self.msg)
			self.pub.publish(Observations(time.time(),obs_msg))
		else:
			self.mismatch_th_counter = self.mismatch_th_counter + 1
			if self.mismatch_th_counter > self.param_mismatch_th:
				#rospy.loginfo('MAtches OUT BOUNDs'+self.param_topic+'Obtainedmean'+str(mean)+' Ubounds='+str(self.delta_Ubound)+'Lbounds='+str(self.delta_Lbound))
				self.msg = '~ok('+self.topic_name+')'
				rospy.loginfo('~ok('+self.topic_name+')MatchCounter')
				obs_msg.append(self.msg)
				self.pub.publish(Observations(time.time(),obs_msg))
			else:
				self.msg = 'ok('+self.topic_name+')'
				#rospy.loginfo('ok('+self.topic_name+')')
				obs_msg.append(self.msg)
				self.pub.publish(Observations(time.time(),obs_msg))
						
=======
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

>>>>>>> safdar
    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s/float(self.param_ws)

    def check_topic(self,top_name,sleeptime,*args):
			try:
				while not rospy.is_shutdown():
						#rospy.loginfo('------------------------------------------------------------------')
						t = 0
						m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
						pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
						#print topicList, top_name
						for item in topicList:
<<<<<<< HEAD
							#print item[0][1:],top_name
							if item[0][1:] == top_name:
									#rospy.loginfo(top_name+'checked against'+item[0][1:])
								t = 1
									#self.lock.acquire()
								if self.incall == False:
									diff = time.time() - self.prev_t
									#rospy.loginfo('diff='+str(diff)+', inclass ='+str(self.incall))
									if (diff > 10*self.delta_Ubound):
								#		rospy.loginfo('~ok('+self.topic_name+')ThTimeOut, diff='+str(diff)+', delatUBound'+str(self.delta_Ubound))
=======
							#print item[0][1:],string
							if item[0][1:] == string:
									t = 1
									if time.time() - self.last_occ_t > 3:
										rospy.loginfo('~ok('+self.topic_name+')1from Thread')
>>>>>>> safdar
										self.pub.publish(Observations(time.time(),['~ok('+self.topic_name+')']))
									#self.lock.release()
									break
						if t == 0:
								#print top_name, topicList
								t = 1
								self.msg = '~ok('+self.topic_name+')'
<<<<<<< HEAD
								rospy.loginfo('~ok('+self.topic_name+')Thread no topic')
=======
								rospy.loginfo('~ok('+self.topic_name+')2 from Thread')
>>>>>>> safdar
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
      
