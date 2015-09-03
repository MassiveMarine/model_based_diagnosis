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
    def __init__(self, topic, callerid, config):
        # create a filter and lock
        self._filter_lock = Lock()
        self._filter = Filter(Config.get_param(config, 'filter'))

        # necessary stuff to calculate time between cb's
        self._msg_t0 = -1.
        self._msg_tn = 0

        # requirements to handle timeout
        self._event = PluginTimeout(Config.get_param(config, 'timeout'), self.timeout_cb)
        self.max_timeouts = 10 if len(Config.get_param(config, 'callerid')) < 1 else None
        self.remaining_timeouts = self.max_timeouts

        # create a predefined error msg
        self._observer_error = observer_error(type='hz', resource=str(topic + ' ' + str(callerid)))

    def timeout_cb(self):
        # publish error
        if error_pub:
            self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
            self._observer_error.error_msg = resource_error_timeout
            error_pub.publish(self._observer_error)

        self._filter_lock.acquire()
        # reset all necessary things to be ready for restart
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._filter.reset()

        # decrement timeout counter
        if self.max_timeouts is not None:
            self.remaining_timeouts -= 1

        self._filter_lock.release()

    def cb(self, msg):
        curr_rostime = rospy.get_rostime()
        curr = curr_rostime.to_sec()
        self._event.set()

        self._filter_lock.acquire()

        if self._msg_t0 < 0 or self._msg_t0 > curr:
            self._msg_t0 = curr
            self._msg_tn = curr
            self._filter.reset()
        else:
            self._filter.update(curr - self._msg_tn)
            self._msg_tn = curr

        self.remaining_timeouts = self.max_timeouts

        self._filter_lock.release()

    def stop(self):
        self._event.stop()


class HzMergedBase():
    def __init__(self, topic, config):
        callerids = Config.get_param(config, 'callerid')
        self.use_all_bases = True if not len(callerids) else False

        self._states = []
        for state_config in Config.get_param(config, 'states'):
            try:
                self._states.append(HzState(state_config))
            except KeyError as e:
                rospy.logerr(e)

        self._resource_info = resource_info(type='hz', resource=str(topic + ' ' + str(callerids)))
        self._observer_error = observer_error(type='hz', resource=str(topic + ' ' + str(callerids)))

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

    def get_resource_info(self, callerids):
        if not len(callerids):
            self._resource_info.states = []
            self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
            return self._resource_info

        is_first = True
        mean = 0
        deviation = []
        sample_size = 0
        for callerid in callerids.itervalues():
            _mean, _deviation, _sample_size = callerid._filter.get_value()
            print ' ', _mean,  _deviation, _sample_size

            if _mean is None or _sample_size < 2:
                continue

            if is_first:
                mean = _mean
                deviation = _deviation
                sample_size = _sample_size
                is_first = False
            else:
                mean = (mean * _mean) / (mean + _mean)
                deviation = [sum(x) for x in zip(deviation, _deviation)]
                sample_size += _sample_size

        print '>', mean, deviation, sample_size
        self._resource_info.states = self._get_valied_states(mean, deviation)
        self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._resource_info


class HzSubs():
    def __init__(self, config):
        self.topic = Config.get_param(config, 'name')

        self.bases = dict()
        self.bases_lock = Lock()

        self.callerids_config = Config.get_param(config, 'callerids')
        self.merged_bases = []
        self.config_for_unknown = None

        for callerid_config in self.callerids_config:
            try:
                callerid_list = Config.get_param(callerid_config, 'callerid')
                self.merged_bases.append(HzMergedBase(self.topic, callerid_config))
                if not len(callerid_list):
                    self.config_for_unknown = callerid_config
            except KeyError as e:
                rospy.logerr(e)

        self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.cb, queue_size=1)

    def add_callerid(self, callerid):
        print 'new callerid schould be added'

        for callerid_config in self.callerids_config:
            callerid_config_name = callerid_config['callerid']

            if callerid in callerid_config_name:
                print "config for callerid found"
                new_base = HzBase(self.topic, callerid, callerid_config)
                self.bases[callerid] = new_base
                return new_base

            if not len(callerid_config_name):
                print "config for unknown callerid found"
                new_base = HzBase(self.topic, callerid, callerid_config)
                self.bases[callerid] = new_base
                return new_base

        print 'nothing found'
        self.bases[callerid] = None
        return None

    def cb(self, msg):
        current_callerid = msg._connection_header['callerid']
        self.bases_lock.acquire()
        if current_callerid not in self.bases:
            base = self.add_callerid(current_callerid)
        else:
            base = self.bases.get(current_callerid)

        if base:
            base.cb(msg)
        self.bases_lock.release()

    def get_resource_infos(self):
        resource_infos = []

        for merge in self.merged_bases:

            self.bases_lock.acquire()
            if merge.use_all_bases:
                callerids = dict(self.bases)
            else:
                callerids = dict((k, self.bases[k]) for k in merge.callerids if k in self.bases)
            self.bases_lock.release()

            resource_infos += [merge.get_resource_info(callerids)]
            # print resource_infos

        return resource_infos

    def cleanup_dead_callerids(self):
        self.bases_lock.acquire()

        callerids = list(self.bases.iterkeys())

        for callerid in callerids:
            base = self.bases[callerid]
            if base.remaining_timeouts is not None and base.remaining_timeouts < 1:
                self.bases[callerid].stop()
                del self.bases[callerid]

        self.bases_lock.release()


class Hz(PluginBase, PluginThread):
    def __init__(self):
        PluginBase.__init__(self, "hz")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        global error_pub
        error_pub = self.error_pub

    def run(self):

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            resource_data = []
            for sub in self.subs:
                resource_data += sub.get_resource_infos()
                sub.cleanup_dead_callerids()


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