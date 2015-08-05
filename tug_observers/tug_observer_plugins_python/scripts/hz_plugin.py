#!/usr/bin/env python
import rospy
# from tug_observers_python import PluginBase
import tug_observers_python

class PluginHz():
    def __init__(self):
        pass

    def run(self):
        rospy.logerr('PluginBase.run')

if __name__ == "__main__":
    rospy.init_node('PluginHz_test', anonymous=False)

    # data = rospy.get_param('/tug_observer_node/setup')
    # print data
    try:
        rospy.loginfo('hallo')
        # rospy.spin()
        # tug_observers_python.p.PluginBase()
        # import tug_observers_python




    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
