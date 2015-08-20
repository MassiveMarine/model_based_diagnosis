#!/usr/bin/env python

import rospy
from tug_observers_python import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import resource_info, resource_error
from filter import FilterFactory
from threading import Lock


class HzBase():
    def __init__(self, topic, config):
        self.filter = FilterFactory.create_filter(config['filter'])

        self.timeout = 1.05 # need to be in each state
        self.states = []

        self._resource_info = resource_info(type='hz', resource=str(topic + ' ' + config['callerid']))

        self._lock = Lock()
        self.event = PluginTimeout(self.timeout, self.timeout_cb)

        self.msg_t0 = -1.
        self.msg_tn = 0

    def timeout_cb(self):
        rospy.logwarn('timeout of topic')
        self._lock.acquire()
        self.msg_t0 = -1.
        self.msg_tn = 0
        self._lock.release()

    def cb(self, msg):
        curr_rostime = rospy.get_rostime()
        curr = curr_rostime.to_sec()
        self.event.set()

        self._lock.acquire()
        if self.msg_t0 < 0 or self.msg_t0 > curr:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.filter.reset()
        else:
            self.filter.update(curr - self.msg_tn)
            self.msg_tn = curr

        self._lock.release()

    def get_resource_info(self):
        value = self.filter.get_value()
        if value is None:
            self._resource_info.states = [str(None)]
        else:
            self._resource_info.states = [str(1. / self.filter.get_value())]

        self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._resource_info


class HzSubs():
    def __init__(self, config):
        self.topic = config['name']

        self.bases = dict()

        for callerid_config in config['callerids']:
            callerid = callerid_config['callerid']
            print callerid

            callerid_unknown = True if callerid == 'unknown' else False

            self.bases[callerid] = HzBase(self.topic, callerid_config)

        if callerid_unknown:
            self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.pre_cb, queue_size=1)
        else:
            self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.cb, queue_size=1)

    def pre_cb(self, msg):
        self.bases['unknown'].cb(msg)
        self.cb(msg)

    def cb(self, msg):
        base = self.bases.get(msg._connection_header['callerid'])
        if base:
            base.cb(msg)
        from sys import getsizeof
        print getsizeof(msg._buff)

    def get_resource_infos(self):
        return [x.get_resource_info() for x in self.bases.itervalues()]


class Hz(PluginBase, PluginThread):
    def __init__(self):
        PluginBase.__init__(self, "hz")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None

    def run(self):
        if not self.subs:
            rospy.logwarn('No topics to work with!! Plugin will stop now.')
            return

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            resource_data = []
            for sub in self.subs:
                resource_data += sub.get_resource_infos()

            PluginBase.publish_info(self, resource_data)
            rate.sleep()

    def initialize(self, config):

        for topic_config in config['topics']:
            self.subs.append(HzSubs(topic_config))

        self.start()


class hz(Hz):
    pass