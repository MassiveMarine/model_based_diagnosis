#!/usr/bin/env python
##
# laser.py is only a node for testing purpose.
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

import roslib; roslib.load_manifest('tug_ist_diagnosis_observers')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def callback1(data):
    rospy.loginfo(rospy.get_name() + ": 1111111111111 %s" % data.data)

def callback2(data):
    rospy.loginfo(rospy.get_name() + ": 2222222222222 %s" % data.data)

def test_node():
    rospy.init_node('laser_node')
    pub = rospy.Publisher('laser_topic', String)
    rospy.Subscriber("chatter1", String, callback1)
    rospy.Subscriber("chatter2", String, callback2)
    while not rospy.is_shutdown():
        str = "publishing fake laser data.."
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        test_node()
    except rospy.ROSInterruptException: pass

