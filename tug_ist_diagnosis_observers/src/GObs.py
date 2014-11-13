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

import roslib.message
import rospy
import sys
import xmlrpclib
import os
import time
import thread
import re
import traceback
import numpy

from array import array
from std_msgs.msg import String
from tug_ist_diagnosis_msgs.msg import Observations


class General_Observer(object):

    def __init__(self):
        rospy.init_node('GObs_Node', anonymous=True)
        self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self.caller_id = '/script'
        self.obs_msg = []
        self.topic_type = ""
        self.topic_name = ""
        self.msg = ""
        self.prev_t = time.time()
        self.pub = rospy.Publisher('/observations', Observations, queue_size=5)
        self.param_topic = rospy.get_param('~topic', '/Topic')
        self.param_frq = rospy.get_param('~frq', 10)
        self.param_dev = rospy.get_param('~dev', 1)
        self.param_ws = rospy.get_param('~ws', 10)
        self.param_mismatch_th = rospy.get_param('~mismatch_th', 5)
        self.circular_queu = [0.1 for i in xrange(self.param_ws)]
        self.delta_t = [0 for i in xrange(self.param_ws)]
        self.sum = 1
        self.count = 0
        self.mismatch_th_counter = 0
        if self.param_topic[0] == '/':
            self.param_topic = self.param_topic[1:len(self.param_topic)]
        thread.start_new_thread(self.check_topic, (self.param_topic, 1))
        thread.start_new_thread(self.make_output, (self.param_topic, 0.5))

    def start(self):
        if self.param_topic[0] != '/':
            self.param_topic = "/%s" % (self.param_topic)
        self.topic_name = self.param_topic[1:len(self.param_topic)]
        # print self.param_topic, self.param_frq, self.param_dev, self.param_ws
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
                rospy.spin()
            else:
                self.msg = '~ok(' + self.topic_name + ')'
                self.pub.publish(Observations(time.time(), [self.msg]))
            time.sleep(1)

    def callback(self, data):
        self.delta_t.pop(0)
        curr_t = time.time()
        delta_t = curr_t - self.prev_t
        #self.sum = self.sum + delta_t
        #self.count = self.count + 1
        self.delta_t.append(delta_t)
        #avg_delta_t = self.average_delta_t()
        #calculated_freq = 1/float(avg_delta_t)
        # print calculated_freq, avg_delta_t
        #diff_freq = abs(self.param_frq - calculated_freq )
        # self.make_output(diff_freq)
        self.prev_t = curr_t

    def make_output(self, diff_freq, delay, *args):
        self.sum = 1
        while not rospy.is_shutdown():
            time.sleep(delay)
            s = numpy.sum(self.delta_t)
            #c = self.count
            T = float(s) / self.param_ws
            if T == 0:
                T = 1
            f = 1 / T
            print f, self.param_frq, abs(self.param_frq - f), self.param_dev, self.param_dev - abs(self.param_frq - f)
            obs_msg = []
            if abs(self.param_frq - f) < self.param_dev:
                if self.mismatch_th_counter > 0:
                    self.mismatch_th_counter = self.mismatch_th_counter - 1
                self.msg = 'ok(' + self.topic_name + ')'
                rospy.logdebug('ok(' + self.topic_name + ')')
                obs_msg.append(self.msg)
                self.pub.publish(Observations(time.time(), obs_msg))
            else:
                self.mismatch_th_counter = self.mismatch_th_counter + 1
                if self.mismatch_th_counter > self.param_mismatch_th:
                    self.msg = '~ok(' + self.topic_name + ')'
                    rospy.logdebug('~ok(' + self.topic_name + ')')
                    obs_msg.append(self.msg)
                    self.pub.publish(Observations(time.time(), obs_msg))
                else:
                    self.msg = 'ok(' + self.topic_name + ')'
                    rospy.logdebug('ok(' + self.topic_name + ')')
                    obs_msg.append(self.msg)
                    self.pub.publish(Observations(time.time(), obs_msg))
            #self.sum = 0
            #self.count = 0

    def average_delta_t(self):
        s = 0
        for val in self.circular_queu:
            s = s + val
        return s / float(self.param_ws)

    def check_topic(self, string, sleeptime, *args):
        try:
            while not rospy.is_shutdown():
                t = 0
                m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
                pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
                # print topicList
                for item in topicList:
                    # print item[0][1:],string
                    if item[0][1:] == string:
                        t = 1
                        if time.time() - self.prev_t > 2:
                            rospy.logdebug('~ok(' + self.topic_name + ')')
                            self.pub.publish(Observations(time.time(), ['~ok(' + self.topic_name + ')']))
                        break
                if t == 0:
                    t = 1
                    self.msg = '~ok(' + self.topic_name + ')'
                    rospy.logdebug('~ok(' + self.topic_name + ')')
                    self.pub.publish(Observations(time.time(), [self.msg]))
                time.sleep(sleeptime)  # sleep for a specified amount of time.
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
