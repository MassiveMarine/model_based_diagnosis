#!/usr/bin/env python

import rospy
from threading import Thread, Event
from tug_observers_msgs.msg import observer_info


class PluginBase():
    """
    Base class for plugins. It lists all required defines.
    """
    def __init__(self, plugin_type):
        """
        Constructor for base plugin.
        :param plugin_type: Name of the type
        """
        self.type = plugin_type

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

        self._stop_request = False

        self.start()

    def run(self):
        """
        Thread runs in here, till shutdown or stop request.
        """
        while not rospy.is_shutdown():
            result = self._event.wait(self._timeout)
            if self._stop_request:
                break
            if result:
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

    def stop(self):
        """
        Kill this thread.
        """
        self._stop_request = True
        self._event.set()

