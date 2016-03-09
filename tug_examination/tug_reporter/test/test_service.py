#!/usr/bin/env python

from tug_reporter.srv import SetString
import rospy
from tug_reporter.srv._SetString import SetStringResponse


def handle_call(req):
    print "service is called with '%s'" % req.message
    return SetStringResponse()

if __name__ == "__main__":
    rospy.init_node('simple_test_servic')
    s = rospy.Service('/test_service', SetString, handle_call)
    print "Test service is ready."
    rospy.spin()
