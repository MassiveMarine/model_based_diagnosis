#!/usr/bin/env python

##
# BiQObs.py is a binary qualitative observer.
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

# The Binary Qualitative Observer observers a trends of two particular values of messages on a two topics
# and publishes match or mismatch between them.

import roslib.message
import rospy
import sys
import xmlrpclib
import os
import time
import numpy as np
import math
import time
import thread

from tug_ist_diagnosis_msgs.msg import Observations


class Regression(object):

    def __init__(self, ws, pb, nb):
        self.s = [[], [], [], []]
        self.t = []
        self.prev_value = 0
        self.ws = ws
        self.n = None
        self.pb = pb
        self.nb = nb
        self.choice = 'n'
        self.value = 0

    def show(self):
        print self.s

    def find(self, value, time, ch):
        self.choice = ch
        self.value = value
        self.s[0].append(value)
        self.s[1].append(value)
        self.s[2].append(value)
        self.s[3].append(value)
        self.t.append(time)
        trend = self.find_slope(self.pb, self.nb)
        return trend

    def find_slope(self, pb, nb):
        if self.choice == 'y':
            r1 = self.linear_regression(self.ws, self.s[0], self.t)
        else:
            r1 = self.value
        self.s[1].pop()
        self.s[1].append(r1)
        r2 = self.linear_regression(self.ws, self.s[1], self.t)
        self.s[2].pop()
        self.s[2].append(r2)
        r3 = self.linear_regression(self.ws, self.s[2], self.t)
        self.s[3].pop()
        self.s[3].append(r3)
        self.remove_tails()
        if r1 < nb:
            self.prev_value = -1
            return -1
        elif r1 > pb:
            self.prev_value = +1
            return +1
        elif ~(((r2 > nb) & (r2 < pb)) & ((r3 > nb) & (r3 < pb))):
            self.prev_value = 0
            return 0
        else:
            return self.prev_value

    def remove_tails(self):
        i = 0
        while(i < (len(self.s[0]) - self.n)):
            self.s[0].pop(0)
            self.s[1].pop(0)
            self.s[2].pop(0)
            self.s[3].pop(0)
            self.t.pop(0)

    def linear_regression(self, ws, s, t):
        Sum_xy = 0.0
        Sum_x = 0.0
        Sum_y = 0.0
        Sum_xx = 0.0
        last_indx = len(t) - 1
        i = last_indx
        n = 0
        while (i > -1) & ((t[last_indx] - t[i]) < ws):
            Sum_xy = Sum_xy + t[i] * s[i]
            Sum_x = Sum_x + t[i]
            Sum_y = Sum_y + s[i]
            Sum_xx = Sum_xx + t[i] * t[i]
            i = i - 1
            n = n + 1

        self.n = n
        if (n * Sum_xx - (Sum_x * Sum_x)) <> 0:
            slope = (n * Sum_xy - Sum_x * Sum_y) / \
                    (n * Sum_xx - (Sum_x * Sum_x))
            return slope
        else:
            return 0


class Qualitative_Observer(object):

    def __init__(self):
        rospy.init_node('BinaryQObs', anonymous=True)
        self.caller_id = '/script'
        self.m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self.param_field1 = rospy.get_param('~field1', 'pose.pose.position.x')
        self.param_topic1 = rospy.get_param('~topic1', '/pose')
        self.param_ws1 = rospy.get_param('~ws1', 1000)
        self.param_pb1 = rospy.get_param('~pb1', 0.0000005)
        self.param_nb1 = rospy.get_param('~nb1', -0.0000005)
        self.param_field2 = rospy.get_param('~field2', 'pose.pose.position.y')
        self.param_topic2 = rospy.get_param('~topic2', '/pose')
        self.param_ws2 = rospy.get_param('~ws2', 1000)
        self.param_pb2 = rospy.get_param('~pb2', 0.0000005)
        self.param_nb2 = rospy.get_param('~nb2', -0.0000005)
        self.mismatch_thr = rospy.get_param('~thr', 30)
        self.mode = rospy.get_param('~mode', 'LIN')
        reg_param = rospy.get_param('~reg', 'yy')
        if len(reg_param) != 2:
            print 'r1 parameter should be nn/yy/ny/yn'
            sys.exit(os.EX_USAGE)
        else:
            self.r11 = reg_param[0]
            self.r12 = reg_param[1]
        self.ws1 = float(self.param_ws1) / 1000.0
        self.ws2 = float(self.param_ws2) / 1000.0
        self.topic1 = ""
        self.topic1_type = ""
        self.topic2 = ""
        self.topic2_type = ""
        self.Trend1 = None
        self.Trend2 = None
        self.data = None
        self.num1 = 0
        self.num2 = 0
        self.sum1 = 0
        self.sum2 = 0
        self.queu = [[0.0 for i in xrange(100)], [0.0 for i in xrange(100)]]
        self.pub = rospy.Publisher('/observations', Observations, queue_size=5)
        self.curr_t1 = None
        self.init_t = time.time()
        self.prev_t = time.time()
        self.prev_data = 0
        self.curr_t2 = None
        self.mismatch = 0
        thread.start_new_thread(self.check_thread, (self.param_topic1, self.param_topic2, 0.2))

    def start(self):
        self.params1 = []
        field = self.param_field1
        i = field.count('.')
        for p in xrange(i):
            i = field.find('.')
            self.params1.append(field[0:i])
            field = field[i + 1:len(field)]
        self.params1.append(field)

        self.params2 = []
        field = self.param_field2
        i = field.count('.')
        for p in xrange(i):
            i = field.find('.')
            self.params2.append(field[0:i])
            field = field[i + 1:len(field)]
        self.params2.append(field)

        topic1_found = False
        topic2_found = False
        firstcheck = True
        while firstcheck:
            pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
            for item in topicList:
                if item[0] == self.param_topic1:
                    self.topic1 = item[0]
                    self.topic1_type = item[1]
                    topic1_found = True
                if item[0] == self.param_topic2:
                    self.topic2 = item[0]
                    self.topic2_type = item[1]
                    topic2_found = True
            if topic1_found == True & topic2_found == True:
                firstcheck = False
                msg_class1 = roslib.message.get_message_class(self.topic1_type)
                msg_class2 = roslib.message.get_message_class(self.topic2_type)
                if self.mode == 'DRV1':
                    rospy.Subscriber(self.topic1, msg_class1, self.drv_call_back)
                    rospy.Subscriber(self.topic2, msg_class2, self.call_back2)
                    self.regression1 = Regression(self.ws1, self.param_pb1, self.param_nb1)
                    self.regression2 = Regression(self.ws2, self.param_pb2, self.param_nb2)
                else:
                    rospy.Subscriber(self.topic1, msg_class1, self.call_back1)
                    rospy.Subscriber(self.topic2, msg_class2, self.call_back2)
                    self.regression1 = Regression(self.ws1, self.param_pb1, self.param_nb1)
                    self.regression2 = Regression(self.ws2, self.param_pb2, self.param_nb2)
                thread.start_new_thread(self.BQObs_thread, (self.param_topic1, self.param_topic2, 0.2))
                rospy.spin()
            time.sleep(1)

    def call_back1(self, data):
        self.curr_t1 = time.time() - self.init_t
        curr_data = self.extract_data(data, self.params1)
        self.sum1 = self.sum1 + curr_data
        self.num1 = self.num1 + 1

    def drv_call_back(self, data):
        self.curr_t1 = time.time() - self.init_t
        curr_t = time.time()
        delta_t = curr_t - self.prev_t
        curr_data = self.extract_data(data, self.params1)
        delta_data = curr_data - self.prev_data
        self.prev_data = curr_data
        self.prev_t = curr_t
        drv = delta_data / delta_t
        self.sum1 = self.sum1 + drv
        self.num1 = self.num1 + 1

    def call_back2(self, data):
        self.curr_t2 = time.time() - self.init_t
        curr_data = self.extract_data(data, self.params2)
        self.sum2 = self.sum2 + curr_data
        self.num2 = self.num2 + 1

    def extract_data(self, data, params):
        c = 0
        while (c < len(params)):
            for f in data.__slots__:
                if f == params[c]:
                    data = getattr(data, f)
                    break
            c = c + 1
        return data

    def publish_output(self):
        if self.topic1[0] == '/':
            self.topic1 = self.topic1[1:len(self.topic1)]
        if self.topic2[0] == '/':
            self.topic2 = self.topic2[1:len(self.topic2)]
        obs_msg = []
        if self.Trend1 == self.Trend2:
            if self.mismatch > 0:
                self.mismatch = self.mismatch - 1
        else:
            self.mismatch = self.mismatch + 1

        # rospy.loginfo('mismatch='+str(self.mismatch)+',thr='+str(self.mismatch_thr))
        field1 = self.param_field1.replace('.', '_')
        field2 = self.param_field2.replace('.', '_')
        if self.mismatch <= self.mismatch_thr:
            rospy.logdebug('matched(' + self.topic1 + ',' + field1 + ',' + self.topic2 + ',' + field2 + ')')
            obs_msg.append('matched(' + self.topic1 + ',' + field1 + ',' + self.topic2 + ',' + field2 + ')')
            self.pub.publish(Observations(time.time(), obs_msg))
        else:
            rospy.logdebug('~matched(' + self.topic1 + ',' + field1 + ',' + self.topic2 + ',' + field2 + ')')
            obs_msg.append('~matched(' + self.topic1 + ',' + field1 + ',' + self.topic2 + ',' + field2 + ')')
            self.pub.publish(Observations(time.time(), obs_msg))

    def BQObs_thread(self, topic1, topic2, sleeptime, *args):
        while True:
            t1 = 0
            t2 = 0
            pubcode, statusMessage, topicList = self.m.getPublishedTopics(self.caller_id, "")
            for item in topicList:
                if item[0] == topic1:
                    t1 = 1
                if item[0] == topic2:
                    t2 = 1

            if t1 == 0:
                t1 = 1
                self.pub.publish(Observations(time.time(),
                                 ['~matched(' + topic1[1:len(topic1)] + ',' + topic2[1:len(topic2)] + ')']))
            if t2 == 0:
                t2 = 1
                self.pub.publish(Observations(time.time(),
                                 ['~matched(' + topic1[1:len(topic1)] + ',' + topic2[1:len(topic2)] + ')']))
            print('Num1=' + str(self.num1) + ',Num2' + str(self.num2))
            if (self.num1 != 0) & (self.num2 != 0):
                data1 = self.sum1 / self.num1
                data2 = self.sum2 / self.num2
                self.Trend1 = self.regression1.find(data1, self.curr_t1, self.r11)
                self.Trend2 = self.regression2.find(data2, self.curr_t2, self.r12)
                rospy.logdebug(self.topic1 + '_avg_data=' + str(data1) + ',' + self.topic1
                                 + 'Trend=' + str(self.Trend1) + ',' + self.topic2
                                 + 'Trend=' + str(self.Trend2) + ',' + self.topic2 + '_avg_data=' + str(data2))
                self.num1 = 0
                self.num2 = 0
                self.sum1 = 0.0
                self.sum2 = 0.0
            self.publish_output()
            time.sleep(sleeptime)  # sleep for a specified amount of time.

    def check_thread(self, topic1, topic2, sleeptime, *args):
        try:
            while not rospy.is_shutdown():
                t1 = 0
                t2 = 0
                m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
                pubcode, statusMessage, topicList = m.getPublishedTopics(self.caller_id, "")
                for item in topicList:
                    if item[0] == topic1:
                        t1 = 1
                    if item[0] == topic2:
                        t2 = 1
                if t1 == 0 | t2 == 0:
                    t1 = 1
                    self.pub.publish(Observations(time.time(),
                                     ['~matched(' + topic1[1:len(topic1)] + ',' + topic2[1:len(topic2)] + ')']))
                time.sleep(sleeptime)  # sleep for a specified amount of time.
        except:
            print "An unhandled exception occured, here's the traceback!"
            traceback.print_exc()


def report_error():
    print 'rosrun tug_ist_diagnosis_observers BiQObs.py <Topic> <Field_variable> <WindowSize>'
    print 'e.g rosrun tug_ist_diagnosis_observers QObs.py _topic:=/odom _field:=pose.pose.position.x _ws:=1000'
    sys.exit(os.EX_USAGE)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        report_error()
    QObs = Qualitative_Observer()
    QObs.start()
