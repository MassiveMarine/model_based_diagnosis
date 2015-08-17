#!/usr/bin/env python

import rospy
from tug_observers_msgs.msg import observer_error
import threading


class PluginBase():
    def __init__(self, type):
        self.type = type

        self._error_pub = rospy.Publisher('/observers/error', observer_error, queue_size=1)

    def initialize(self, parameter):
        return False


class PluginThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
