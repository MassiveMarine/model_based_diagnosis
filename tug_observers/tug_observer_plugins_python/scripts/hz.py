#!/usr/bin/env python

import rospy
from tug_observers_python import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import resource_info, resource_error
from filter import FilterFactory
from nominal_value import NominalValueFactory
from threading import Lock


class HzState():
    def __init__(self, config):
        self.name = config['state']
        self._frequency = NominalValueFactory.create_nominal_value(config['frequenzy'])

    def is_nominal(self, value):
        if self._frequency is None:
            return False
        return self._frequency.is_nominal(value)


class HzBase():
    def __init__(self, topic, config):
        self._filter = FilterFactory.create_filter(config['filter'])

        self._states = []
        for state_config in config['states']:
            self._states.append(HzState(state_config))

        self._resource_info = resource_info(type='hz', resource=str(topic + ' ' + config['callerid']))

        self._lock = Lock()
        self._event = PluginTimeout(config['timeout'], self.timeout_cb)

        self._msg_t0 = -1.
        self._msg_tn = 0

    def timeout_cb(self):
        rospy.logwarn('timeout of topic')
        self._lock.acquire()
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._filter.reset()
        self._lock.release()

    def cb(self, msg):
        curr_rostime = rospy.get_rostime()
        curr = curr_rostime.to_sec()
        self._event.set()

        self._lock.acquire()
        if self._msg_t0 < 0 or self._msg_t0 > curr:
            self._msg_t0 = curr
            self._msg_tn = curr
            self._filter.reset()
        else:
            self._filter.update(curr - self._msg_tn)
            self._msg_tn = curr

        self._lock.release()

    def _get_valied_states(self, value):
        states = []
        for state in self._states:
            if state.is_nominal(value):
                states.append(state.name)
        return states

    def get_resource_info(self):
        value = self._filter.get_value()
        if value is None:
            self._resource_info.states = []
        else:
            self._resource_info.states = self._get_valied_states(1. / self._filter.get_value())  #[str(1. / self._filter.get_value())]

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