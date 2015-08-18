#!/usr/bin/env python

import rospy
from tug_observers_python import PluginBase, PluginThread
from tug_observers_msgs.msg import resource_info, resource_error

class HzBase():
    def __init__(self, topic):
        self.topic = topic
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

    def get_states(self):
        return ['1']

        # print 1.0 / self.times[-1]


class Hz(PluginBase, PluginThread):
    def __init__(self):
        PluginBase.__init__(self, "hz")
        PluginThread.__init__(self)

        self.subs = []

        self.topics = None

    def run(self):
        print 'running'

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for [plugin, resource_data] in self.subs:
                resource_data.header = rospy.Header(stamp=rospy.Time.now())
                resource_data.states = plugin.get_states()

            PluginBase.publish_info(self, [x[1] for x in self.subs])
            rate.sleep()

        print 'stopped'

    def initialize(self, topics):
        for topic in topics:
            self.subs.append([HzBase(topic), resource_info(type='hz', resource=topic)])

        self.start()
#
#
# if __name__ == "__main__":
#     rospy.init_node('tug_Hz', anonymous=False)
#     try:
#         rospy.loginfo("starting " + rospy.get_name())
#
#         test = Hz()
#
#         rospy.spin()
#
#     except KeyboardInterrupt:
#         pass
#     except rospy.ROSInterruptException:
#         pass
#
#     finally:
#         rospy.logwarn( 'Hz stopped')
#


