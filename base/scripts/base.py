#!/usr/bin/env python

import rospy
import roslib

class BaseClass:
    def __init__(self):
        self.name = 'test_fct_base_class'
        pass

    def run(self):
        pass

if __name__ == "__main__":
    rospy.init_node('base', anonymous=False)

    try:
        rospy.logerr('base started')

        import rospkg
        import roslib
        import sys

        rospack = rospkg.RosPack()
        pkg = 'test1'

        new_pkg_path = rospack.get_path(pkg) + '/scripts'
        sys.path = [new_pkg_path] + sys.path
        import testscript

        # mod = __import__('testscript')
        csadfsd = testscript.TestClass()
        csadfsd.run()
        # print p_module.split('.')[1:]

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

