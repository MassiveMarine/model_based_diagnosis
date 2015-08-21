#!/usr/bin/env python

import rospy
from threading import Thread, Event
from tug_observers_msgs.msg import observer_error, observer_info


class PluginBase():
    """
    Base calss for plugins. It lists all required defines.
    """
    def __init__(self, type):
        """
        Constructor for base plugin.
        :param type: Name of the type
        """
        self.type = type

        self.error_pub = rospy.Publisher('/observers/error', observer_error, queue_size=1)
        self.info_pub = rospy.Publisher('/observers/info', observer_info, queue_size=1)

    def initialize(self, config):
        """
        Called for each plugin to set it up depending on the given config.
        :param config: Configuration from yaml file
        """
        pass


class PluginThread(Thread):
    """
    This should be used if plugin uses a main thread.
    """
    def __init__(self):
        Thread.__init__(self)
        self.setDaemon(True)


class PluginTimeout(Thread):
    """
    This is used to call a function if 'set' is not called in time.
    """
    def __init__(self, timeout, callback):
        """
        Constructor for the plugin timeout.
        :param timeout: time to wait for event in seconds
        :type timeout: float
        :param callback: function that should be called if timeout is reached
        """
        Thread.__init__(self)

        self._timeout = timeout
        self._callback = callback
        self.setDaemon(True)

        # from threading import Event
        self._event = Event()
        self._event.clear()
        self.start()

    def run(self):
        """
        Thread runs in here, till shutdown.
        """
        while not rospy.is_shutdown():
            if self._event.wait(self._timeout):
                self._event.clear()
            else:
                try:
                    self._callback()
                except TypeError as e:
                    rospy.logerr(str(self.__class__) + str(e))

    def set(self):
        """
        Stop timer.
        """
        self._event.set()

    def clear(self):
        """
        Start new timeout.
        """
        self._event.clear()

