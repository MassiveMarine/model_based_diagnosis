#!/usr/bin/env python

from threading import Lock

import rospy

from tug_observers import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import observer_error, observer_info, resource_info, resource_error
from tug_observer_plugin_utils import Filter, SingleValueHypothesisCheckFactory
from tug_python_utils import YamlHelper as Config


max_timeouts_in_a_row = 10
error_pub = None

# predefined resource error msgs that are used if a error is published
resource_error_timeout = resource_error(error_msg='Timeout',
                                        verbose_error_msg='Timeout of Topic',
                                        error=resource_error.NO_AVAILABLE)
resource_error_no_state_fits = resource_error(error_msg='No State fits',
                                              verbose_error_msg='No state can be found for the measured results',
                                              error=resource_error.NO_STATE_FITS)


class HzState():
    """
    This class is used for hypothesis checks for a state. Each state
    has one or more hypotheses. These are managed in here.
    """
    def __init__(self, config):
        """
        Constructor of a new state. It reads the config used for this state and create a new hypothesis check instance.
        :param config: Configuration from yaml file
        """
        self.name = Config.get_param(config, 'state')
        frequency = Config.get_param(config, 'frequenzy')
        self.hypothesis_check = SingleValueHypothesisCheckFactory.create_single_value_hypothesis_check(frequency)

    def check_hypothesis(self, value, deviation, sample_size):
        """
        Forward the information for hypothesis check.
        :param value: mean value which should be checked
        :param deviation: deviation values which should be checked
        :param sample_size: number of samples that are used for mean and deviation
        :return: result of the hypothesis check
        """
        return self.hypothesis_check.check_hypothesis(value, deviation, sample_size)


class HzBase():
    """
    This class is used for each callerid. Each instance of this has its own timeout thread.
    The delay calculation will be done here.
    """
    def __init__(self, topic, callerid, config):
        """
        Constructor of the HzBase class. It manages the filter, creates and starts the timeout-thread
        and initialize stuff for the delay calculation.
        :param topic: name of topic that is subscribed
        :param callerid: name of the node that publish on the topic
        :param config: Configuration from yaml file
        """
        # create a filter and lock
        self._filter_lock = Lock()
        self._filter = Filter(Config.get_param(config, 'filter'))

        # necessary stuff to calculate time between cb's
        self._msg_t0 = -1.
        self._msg_tn = 0

        # requirements to handle timeout
        self._event = PluginTimeout(Config.get_param(config, 'timeout'), self.timeout_cb)
        self.max_timeouts = max_timeouts_in_a_row if len(Config.get_param(config, 'callerid')) < 1 else None
        self.remaining_timeouts = self.max_timeouts

        # create a predefined error msg
        self._observer_error = observer_error(type='hz', resource=str(topic + ' ' + str(callerid)))

    def timeout_cb(self):
        """
        Callback method that is called if a timeout is reached.
        It will reset all information for delay calculation and the filter.
        """

        self._filter_lock.acquire()
        # reset all necessary things to be ready for restart
        self._msg_t0 = -1.
        self._msg_tn = 0
        self._filter.reset()

        # decrement timeout counter
        if self.max_timeouts is not None:
            self.remaining_timeouts -= 1
            print 'self.remaining_timeouts: ', self.remaining_timeouts

        self._filter_lock.release()

        # publish error
        if error_pub and self.remaining_timeouts >= 0:
            self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
            self._observer_error.error_msg = resource_error_timeout
            error_pub.publish(self._observer_error)

    def cb(self, msg):
        """
        Callback method that is called because of the forwarding if HzSub.
        Calculates the delay between this and the previous callback call and updates the filter.
        :param msg: message from publisher
        """
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
        """
        Stop the observation of this callerid. This will stop the Timeout thread.
        """
        self._event.stop()

    def get_values(self):
        """
        Get the information of the filter of this callerid
        :return information of the filter of this callerid
        """
        return self._filter.get_value()

    def is_in_timeout(self):
        """
        Check if a publish of the callerid is overdue
        :return True if timeout is reached currently, otherwise False.
        """
        return True if self.remaining_timeouts < self.max_timeouts else False


class HzMergedBase():
    """
    This class is used to manage the combination of callerids and its possible states.
    The
    """
    def __init__(self, topic, config):
        """
        Constructor that defines which callerids should be merged. Also all states for this combination
        of callerids are defined depending on the config.
        :param topic: name of topic that is subscribed
        :param config: Configuration from yaml file
        """
        # public variables
        self.callerids = Config.get_param(config, 'callerid')
        self.use_all_bases = True if not len(self.callerids) else False

        # Try to load and setup all states as defined in the yaml-file
        self._states = []
        for state_config in Config.get_param(config, 'states'):
            try:
                self._states.append(HzState(state_config))
            except KeyError as e:
                rospy.logerr(e)

        # create a predefined msgs for info and error
        self._resource_info = resource_info(type='hz', resource=str(topic + ' ' + str(self.callerids)))
        self._observer_error = observer_error(type='hz', resource=str(topic + ' ' + str(self.callerids)))

    def get_resource_info(self, callerids):
        """
        Generate a resource information for publishing by readout and combine the given callerids and
        make hypotheses checks.
        :param callerids: objects of all callerids that should be combined
        :return a resource information that can be published
        """

        if not len(callerids):
            self._resource_info.states = []
            self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
            return self._resource_info

        def merge_callerids(callerids_to_merge):
            """
            Iterate over all callerids and combine them.
            :param callerids_to_merge: objects of all callerids that should be combined
            :return: merged mean, deviation and number of samples
            """
            # prepare for combining callerids
            is_first = True
            mean = 0
            deviation = []
            sample_size = 0

            for callerid in callerids_to_merge.itervalues():

                _mean, _deviation, _sample_size = callerid.get_values()

                # check if the current callerid is valid and contains valid information
                if callerid.is_in_timeout():
                    continue
                if _mean is None or _sample_size < 2:
                    continue

                # sum up all information of all callerids
                if is_first:
                    mean = _mean
                    deviation = _deviation
                    sample_size = _sample_size
                    is_first = False
                else:
                    mean = (mean * _mean) / (mean + _mean)
                    deviation = [sum(x) for x in zip(deviation, _deviation)]
                    sample_size += _sample_size

            return mean, deviation, sample_size

        def get_valid_states(value, deviation, sample_size):
            """
            Check hypotheses of all states.
            :param value: mean value which should be checked
            :param deviation: deviation values which should be checked
            :param sample_size: number of samples that are used for mean and deviation
            """
            states = []
            for state in self._states:
                try:
                    if state.check_hypothesis(value, deviation, sample_size):
                        states.append(state.name)
                except AttributeError as e:
                    rospy.logerr(e)

            if not states:
                self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
                self._observer_error.error_msg = resource_error_no_state_fits
                error_pub.publish(self._observer_error)
            return states

        # combine callerids
        mean_merged, deviation_merged, sample_size_merged = merge_callerids(callerids)

        # setup predefined resource information
        self._resource_info.states = get_valid_states(mean_merged, deviation_merged, sample_size_merged)
        self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._resource_info


class HzSubs():
    """
    This class is used to subscribe to a topic, process the information for each callerid. It has a list
    which contains objects for each callerid that publish at this topic. To that it also saves all defined
    combinations of callerids that should be merged.
    """
    def __init__(self, config):
        """
        Constructor to create a object of HzSubs that exists per topic. For each combination a HzMergedBase
        object is created. The last callerid-config that defines no callerid is used as default config for
        new callerids that are not explicitly named in the config. If there is no callerid-config that can
        be used as default config, unknown callerids are ignored.
        :param config: Configuration from yaml file
        """
        # topic that should be subscribed
        self._topic = Config.get_param(config, 'name')

        # store all callerids of this topic separately
        self._bases = dict()
        self._bases_lock = Lock()

        # define necessary stuff to handle combinations of callerid
        self._callerids_config = Config.get_param(config, 'callerids')
        self._merged_bases = []
        self._config_for_unknown = None

        # init all states per callerid-combination and detect a possible
        # config that can be used as default config.
        for callerid_config in self._callerids_config:
            try:
                callerid_list = Config.get_param(callerid_config, 'callerid')
                self._merged_bases.append(HzMergedBase(self._topic, callerid_config))
                if not len(callerid_list):
                    self._config_for_unknown = callerid_config
            except KeyError as e:
                rospy.logerr(e)

        # subscribe to given topic
        self.sub = rospy.Subscriber(self._topic, rospy.AnyMsg, self.cb, queue_size=1)

    def add_callerid(self, callerid):
        """
        Add a new object for a new callerid and add it to list
        :param callerid: name of the callerid
        :return: the new object created for the given callerid
        """
        print 'new callerid should be added'

        best_config = self._config_for_unknown
        new_base = None

        for callerid_config in self._callerids_config:
            try:
                if callerid in Config.get_param(callerid_config, 'callerid'):
                    best_config = callerid_config
                    break
            except KeyError as e:
                rospy.logerr(e)

        if best_config:
            new_base = HzBase(self._topic, callerid, self._config_for_unknown)

        self._bases[callerid] = new_base
        return new_base

    def cb(self, msg):
        """
        Callback method that is called by the subscriber.
        After identifying the callerid the corresponding callback is called.
        If the callerid does not exists, it will be created as long a suitable config can be found.
        If there exists a config with undefined callerids, this will be the default config.
        Otherwise the callerid is added to list, but without calling a callback.
        :param msg: message from publisher
        """
        current_callerid = msg._connection_header['callerid']
        self._bases_lock.acquire()
        if current_callerid not in self._bases:
            base = self.add_callerid(current_callerid)
        else:
            base = self._bases.get(current_callerid)

        if base:
            base.cb(msg)
        self._bases_lock.release()

    def get_resource_info(self):
        """
        Create array of resource-info of all defined combinations of callerids.
        """
        info = []

        for merge in self._merged_bases:

            # create reduced list of callerids, that should be merged
            self._bases_lock.acquire()
            if merge.use_all_bases:
                callerids = dict(self._bases)
            else:
                callerids = dict((k, self._bases[k]) for k in merge.callerids if k in self._bases)
            self._bases_lock.release()

            # get resource info of subset of callerids and add it to resource info list
            info += [merge.get_resource_info(callerids)]

        return info

    def cleanup_dead_callerids(self):
        """
        Remove callerids from list if the number of timeouts reaches a maximum, but
        only if its not exactly defined in the config.
        """
        self._bases_lock.acquire()

        callerids = list(self._bases.iterkeys())

        for callerid in callerids:
            base = self._bases[callerid]
            if base and base.remaining_timeouts is not None and base.remaining_timeouts < 1:
                self._bases[callerid].stop()
                del self._bases[callerid]

        self._bases_lock.release()


class Hz(PluginBase, PluginThread):
    """
    Hz main class. All necessary subscribers for the given topics are created.
    A additional thread is also started to publish the observer-info with a defined rate.
    """
    def __init__(self):
        """
        Constructor for Hz main
        """
        PluginBase.__init__(self, "hz")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        global error_pub
        error_pub = self.error_pub
        self.rate = 1

    def run(self):
        """
        Thread runs in here, till shutdown. It will publish observer info with a defined
        rate and also cleanup old/unused callerids of topics.
        """
        while not rospy.is_shutdown():
            resource_data = []
            for sub in self.subs:
                resource_data += sub.get_resource_info()
                sub.cleanup_dead_callerids()

            msg = observer_info(resource_infos=resource_data)
            self.info_pub.publish(msg)
            self.rate.sleep()

    def initialize(self, config):
        """
        Setup depending on the given config. For each topic a 'HzSubs'-object is created.
        In addition it start the main thread of this plugin.
        :param config: Configuration from yaml file
        """
        global max_timeouts_in_a_row
        max_timeouts_in_a_row = Config.get_param(config, 'max_timeouts_in_a_row')

        self.rate = rospy.Rate(Config.get_param(config, 'main_loop_rate'))
        for topic_config in Config.get_param(config, 'topics'):
            try:
                self.subs.append(HzSubs(topic_config))
            except (KeyError, StandardError) as e:
                rospy.logerr(e)

        self.start()


hz = Hz