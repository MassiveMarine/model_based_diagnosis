#!/usr/bin/env python

import rospy
# import roslib
# roslib.load_manifest('rosparam')
import rosparam
import rostopic
import roswtf


class PluginManager:
    def __init__(self):
        pass

    @staticmethod
    def load_plugin(name):
        mod = __import__(name)
        return mod

    def call_plugin(self):
        plugin = self.load_plugin("plugin_base")
        base = plugin.PluginBase()
        base.run()


if __name__ == "__main__":
    rospy.init_node('tug_observer', anonymous=False)

    # data = rospy.get_param('/tug_observer_node/setup')
    # print data
    try:
        # rospy.loginfo('hallo')
        # rospy.spin()

        plugin_manager = PluginManager()
        plugin_manager.call_plugin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

