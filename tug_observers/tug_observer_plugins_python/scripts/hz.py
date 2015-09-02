#!/usr/bin/env python

import rospy
from tug_observers_python import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import observer_error, observer_info, resource_info, resource_error
from filter import Filter
from nominal_value import NominalValueFactory
from threading import Lock

from tug_python_utils import YamlHelper as Config

error_pub = None

resource_error_timeout = resource_error(error_msg='Timeout',
                                        verbose_error_msg='Timeout of Topic',
                                        error=resource_error.NO_AVAILABLE)
resource_error_no_state_fits = resource_error(error_msg='No State fits',
                                              verbose_error_msg='No state can be found for the measured results',
                                              error=resource_error.NO_STATE_FITS)


class HzState():
    def __init__(self, config):
        self.name = Config.get_param(config, 'state')
        frequency = Config.get_param(config, 'frequenzy')
        self._value = NominalValueFactory.create_nominal_value(Config.get_param(frequency, 'value'))
        self._deviation_nominal_values = []
        for deviation_config in Config.get_param(frequency, 'deviation'):
            self._deviation_nominal_values.append(NominalValueFactory.create_nominal_value(deviation_config))

    def is_nominal(self, value, deviation):
        value_nominal = self._value.is_nominal(value)
        if not len(deviation):
            return False

        if len(deviation) is not len(self._deviation_nominal_values):
            rospy.logwarn("Number of deviations do not match with config!")
            return value_nominal

        deviation_nominal = all(o.is_nominal(deviation[c]) for c, o in enumerate(self._deviation_nominal_values))

        return value_nominal and deviation_nominal


class HzBase():
    def __init__(self, topic, config=None):
        if not config:
            self.cb = self.cb_empty
            self.is_dummy_base = True
            return

        self.is_dummy_base = False

        self._filter = Filter(Config.get_param(config, 'filter'))

        self._states = []
        for state_config in Config.get_param(config, 'states'):
            try:
                self._states.append(HzState(state_config))
            except KeyError as e:
                rospy.logerr(e)


        self.topic = topic
        self.callerid = Config.get_param(config, 'callerid')
        self._resource_info = resource_info(type='hz', resource=str(self.topic + ' ' + self.callerid))
        self._observer_error = observer_error(type='hz', resource=str(self.topic + ' ' + self.callerid))

        self._lock = Lock()
        timeout = Config.get_param(config, 'timeout')
        self._event = PluginTimeout(timeout, self.timeout_cb)

        self._msg_t0 = -1.
        self._msg_tn = 0

    def timeout_cb(self):
        # rospy.logwarn('timeout of topic')
        if error_pub:
            self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
            self._observer_error.error_msg = resource_error_timeout
            error_pub.publish(self._observer_error)
        self._lock.acquire()
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._filter.reset()
        self._lock.release()

    def cb_empty(self, msg):
        print 'empty cb'
        pass

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

    def _get_valied_states(self, value, deviation=[]):
        states = []
        for state in self._states:
            if state.is_nominal(value, deviation):
                states.append(state.name)

        if not states:
            self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
            self._observer_error.error_msg = resource_error_no_state_fits
            error_pub.publish(self._observer_error)
        return states

    def get_resource_info(self):
        mean, deviation, sample_size = self._filter.get_value()
        if mean is None or sample_size < 2:
            self._resource_info.states = []
        else:
            print mean,  deviation, sample_size
            self._resource_info.states = self._get_valied_states(mean, deviation)

        self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._resource_info


class HzSubs():
    def __init__(self, config):
        self.topic = Config.get_param(config, 'name')

        self.bases = dict()

        self.callerids_config = Config.get_param(config, 'callerids')

        # for callerid_config in callerids:
        #     try:
        #         callerid = Config.get_param(callerid_config, 'callerid')
        #         callerid_unknown = True if callerid == 'unknown' else False
        #         self.bases[callerid] = HzBase(self.topic, callerid_config)
        #     except KeyError as e:
        #         rospy.logerr(e)

        # if not len(self.bases):
        #     raise StandardError("No Subscriber possible for topic '" + self.topic + "' maybe because of errors in config!")
        # if callerid_unknown:
        #     self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.pre_cb, queue_size=1)
        # else:
        #     self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.cb, queue_size=1)
        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.cb, queue_size=1)

    def add_callerid(self, callerid):
        print 'new callerid schould be added'

        callerid_unknown = False

        for callerid_config in self.callerids_config:
            callerid_config_name = callerid_config['callerid']
            print callerid_config_name, callerid
            # if callerid_config_name == 'unknown':
            #     callerid_unknown = True

            if callerid_config_name == callerid:
                print "config for callerid found"
                new_base = HzBase(self.topic, callerid_config)
                self.bases[callerid] = new_base
                return new_base

        # if callerid_unknown:
        #     print "a 'unknown' is in list"
        #     return None

        print 'nothing found'
        new_base = HzBase(self.topic, None)
        self.bases[callerid] = new_base
        return new_base

    # def pre_cb(self, msg):
    #     self.bases['unknown'].cb(msg)
    #     self.cb(msg)

    def cb(self, msg):
        current_callerid = msg._connection_header['callerid']
        print 'current callerid: ' + str(current_callerid)

        base = self.bases.get(msg._connection_header['callerid'])

        if base:
            print 'callerid is in list'
            base.cb(msg)
        else:
            print 'callerid is NOT in list'
            new_base = self.add_callerid(current_callerid)
            new_base.cb(msg)

    def get_resource_infos(self):
        return [x.get_resource_info() for x in self.bases.itervalues() if not x.is_dummy_base]


class Hz(PluginBase, PluginThread):
    def __init__(self):
        PluginBase.__init__(self, "hz")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        global error_pub
        error_pub = self.error_pub

    def run(self):
        if not self.subs:
            rospy.logwarn('No topics to work with!! Plugin will stop now.')
            return

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            resource_data = []
            for sub in self.subs:
                resource_data += sub.get_resource_infos()

            msg = observer_info(resource_infos=resource_data)
            self.info_pub.publish(msg)
            rate.sleep()

    def initialize(self, config):

        for topic_config in config['topics']:
            try:
                self.subs.append(HzSubs(topic_config))
            except (KeyError, StandardError) as e:
                rospy.logerr(e)

        self.start()


class hz(Hz):
    pass