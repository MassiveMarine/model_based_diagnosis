#!/usr/bin/env python

import rospy
import threading
from tug_observers_msgs.msg import observer_error, observer_info


class PluginBase():
    def __init__(self, type):
        self.type = type

        self._error_pub = rospy.Publisher('/observers/error', observer_error, queue_size=1)
        self._info_pub = rospy.Publisher('/observers/info', observer_info, queue_size=1)

    def initialize(self, parameter):
        return False

    def publish_info(self, resource_infos):
        msg = observer_info(resource_infos=resource_infos)
        self._info_pub.publish(msg)
        pass

    def publish_error(self):
        pass


class PluginThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
