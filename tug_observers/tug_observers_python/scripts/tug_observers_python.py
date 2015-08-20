#!/usr/bin/env python

import rospy
from threading import Thread, Event
from tug_observers_msgs.msg import observer_error, observer_info


class PluginBase():
    def __init__(self, type):
        self.type = type

        self._error_pub = rospy.Publisher('/observers/error', observer_error, queue_size=1)
        self._info_pub = rospy.Publisher('/observers/info', observer_info, queue_size=1)


    def initialize(self, config):
        return False

    def publish_info(self, resource_infos):
        msg = observer_info(resource_infos=resource_infos)
        self._info_pub.publish(msg)
        pass

    def publish_error(self):
        pass


class PluginThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.setDaemon(True)


class PluginTimeout(Thread):
    def __init__(self, timeout, callback):
        Thread.__init__(self)

        self._timeout = timeout
        self._callback = callback
        self.setDaemon(True)

        # from threading import Event
        self._event = Event()
        self._event.clear()
        self.start()

    def run(self):
        while not rospy.is_shutdown():
            if self._event.wait(self._timeout):
                self._event.clear()
            else:
                try:
                    self._callback()
                except TypeError as e:
                    rospy.logerr(str(self.__class__) + str(e))

    def set(self):
        self._event.set()
        pass

    def clear(self):
        self._event.clear()
        pass

