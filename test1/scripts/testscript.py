#!/usr/bin/env python

import rospy
from base import BaseClass

class TestClass(BaseClass):
    def __init__(self):
        BaseClass.__init__(self)

    def run(self):
        print 'running'
        rospy.spin()

# rospy.init_node('test', anonymous=False)
# test_class = TestClass()
# test_class.run()