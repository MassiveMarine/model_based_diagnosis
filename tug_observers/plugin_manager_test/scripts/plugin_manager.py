#!/usr/bin/env python

# basics
import rospy

import threading
from tug_observers_msgs.msg import observer_error

class PluginBase(threading.Thread):
    def __init__(self, type):
        threading.Thread.__init__(self)
        self.type = type

        self._error_pub = rospy.Publisher('/observers/error', observer_error, queue_size=1)

    def initialize(self, parameter):
        return False

    def run(self):
        pass


class HzBase():
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, rospy.AnyMsg, self.cb, queue_size=1)
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.times = []

    def cb(self, msg):
        curr_rostime = rospy.get_rostime()

        # time reset
        if curr_rostime.is_zero():
            if len(self.times) > 0:
                print("time has reset, resetting counters")
                self.times = []
            return

        curr = curr_rostime.to_sec()
        if self.msg_t0 < 0 or self.msg_t0 > curr:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.times = []
        else:
            self.times.append(curr - self.msg_tn)
            self.msg_tn = curr


class Hz(PluginBase):
    def __init__(self):
        PluginBase.__init__(self, "hz")

        self.subs = []

        self.topics = None

    def initialize(self, topics):
        for topic in topics:
            self.subs.append(HzBase(topic))

    def run(self):
        print 'running'
        rospy.spin()


class PluginManager():
    def __init__(self):
        self._plugins = []

    def load_plugin(self, type):
        # PluginType = __import__(type)
        # plugin = PluginType()
        plugin = globals()[type]()

        self._plugins.append(plugin)
        return plugin

    def get_plugin_list(self):
        return list(self._plugins)

    def wait_for_all_plugins(self):
        for plugin in self._plugins:
            if plugin.isAlive():
                plugin.join()


# class ObserverNode():
#     def __init__(self):
#         pass
#
#     def init_plugins(self):
#         pass


if __name__ == "__main__":
    rospy.init_node('plugin_test', anonymous=False)

    try:
        rospy.loginfo("starting " + rospy.get_name())

        manager = PluginManager()
        new_plugin = manager.load_plugin('Hz')
        new_plugin.initialize(['/test'])

        for plugin in manager.get_plugin_list():
            plugin.start()

        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        manager.wait_for_all_plugins()