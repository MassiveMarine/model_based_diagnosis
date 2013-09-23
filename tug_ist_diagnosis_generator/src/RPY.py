#!/usr/bin/env python
import roslib.message;roslib.load_manifest('tug_ist_diagnosis_generator')
import rospy
import sys
import xmlrpclib
import os
import tf
#from tf import TransformListener


class RPY(object):
	def __init__(self):
		rospy.init_node('observers_generator', anonymous=False)
		self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
		self.caller_id = '/script'
		self.topic = rospy.get_param('~topic', 100)
		self.orien = rospy.get_param('~orien', 100)
		
	def start(self):
		self.extract_node()
	def callback(self,msg,topic):
		#print msg.orientation
		#print msg.pose.pose.orientation
		#print '('+str(data.orientation.x)+','+str(data.orientation.y)+','+str(data.orientation.z)+str(data.orientation.w)+')'
		#theta = tf.transformations.euler_from_quaternion(msg.pose.pose.orientation)
		#print len(msg.__slots__)
		for f in msg.__slots__:
			if (('header' in f) | ('frame_id' in f)):
				continue
			if (getattr(msg,f)!=tuple):
				if 'orientation' in getattr(msg,f):
					print f
					break
				else:
					msg = getattr(msg,f)
			
		#	#print f,isinstance(a.__slots__, list)
		#	for f1 in a.__slots__:
		#		b = getattr(a,f1)
		#		if type(b)==tuple:
		#			continue
		#		for f2 in b.__slots__:
		#			print f, f1, f2
		#theta = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		#rospy.loginfo('PY_roll='+str(theta[0])+' PY_pitch='+str(theta[1])+' PY_yaw='+str(theta[2]))
	
	def extract_node(self):
		pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
		for topic in topicList:
			if topic[0] == self.topic:
				msg_class = roslib.message.get_message_class(topic[1])
				rospy.Subscriber(topic[0], msg_class, self.callback, topic[0])
		rospy.spin()

					
if __name__ == '__main__':
	rpy = RPY()
	rpy.start()
