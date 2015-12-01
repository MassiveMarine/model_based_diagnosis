#!/usr/bin/env python

from threading import Lock

import rospy

from tug_observers import PluginBase, PluginThread
from tug_observers_msgs.msg import observer_info, observation_info, observation
from tug_observer_plugin_utils import Filter, SingleValueHypothesisCheckFactory
from tug_python_utils import YamlHelper as Config

import rostopic


# predefined resource error msgs that are used if a error is published
observation_no_state_fits = observation(observation_msg='No State fits',
                                        verbose_observation_msg='No state can be found for the measured results',
                                        observation=observation.NO_STATE_FITS)


class TimestampState():
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
        self.number = Config.get_param(config, 'number')
        age = Config.get_param(config, 'age')
        self.hypothesis_check = SingleValueHypothesisCheckFactory.create_single_value_hypothesis_check(age)

    def check_hypothesis(self, value, deviation, sample_size):
        """
        Forward the information for hypothesis check.
        :param value: mean value which should be checked
        :param deviation: deviation values which should be checked
        :param sample_size: number of samples that are used for mean and deviation
        :return: result of the hypothesis check
        """
        return self.hypothesis_check.check_hypothesis(value, deviation, sample_size)


class TimestampBase():
    """
    This class is used for each callerid.
    """
    def __init__(self, topic, callerid, config):
        """
        Constructor of the TimeoutBase class.
        :param topic: name of topic that is subscribed
        :param callerid: name of the node that publish on the topic
        :param config: Configuration from yaml file
        """
        # create a filter and lock
        self._filter_lock = Lock()
        self._filter = Filter(Config.get_param(config, 'filter'))

        # create a predefined error msg
        # self._observation_info = observation_info(type='timestemp', resource=str(topic + '[' + str(callerid)) + ']')
        self._topic = topic

    def cb(self, msg, curr_rostime):
        """
        Callback method that is called from the forwarding of TimeoutSub.
        Calculates the delay between real ros time and the rostime in the header of the msg and updates the filter.
        :param msg: message from publisher
        """
        self._filter_lock.acquire()
        self._filter.update((curr_rostime - msg.header.stamp.to_nsec()) / 1000000000.0)
        self._filter_lock.release()

    def get_values(self):
        """
        Get the information of the filter of this callerid
        :return information of the filter of this callerid
        """
        # print self._topic, self._filter.get_value()
        return self._filter.get_value()


class TimestampMergedBase():
    """
    This class is used to manage the combination of callerids and its possible states.
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
                self._states.append(TimestampState(state_config))
            except KeyError as e:
                rospy.logerr(e)

        # create a predefined msgs for info and error
        self._observation_info = observation_info(type='timestamp', resource=str(topic + ' ' + str(self.callerids)))

    def get_observation_info(self, callerids):
        """
        Generate a resource information for publishing by readout and combine the given callerids and
        make hypotheses checks.
        :param callerids: objects of all callerids that should be combined
        :return a resource information that can be published
        """

        if not len(callerids):
            self._observation_info.observation = [observation_no_state_fits]
            self._observation_info.header = rospy.Header(stamp=rospy.Time.now())
            return self._observation_info

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
            observations = []
            for state in self._states:
                try:
                    if state.check_hypothesis(value, deviation, sample_size):
                        observations.append(observation(observation_msg=state.name,
                                                        verbose_observation_msg=state.name,
                                                        observation=state.number))
                except AttributeError as e:
                    rospy.logerr(e)

            if not observations:
                observations.append(observation_no_state_fits)

            return observations

        # combine callerids
        mean_merged, deviation_merged, sample_size_merged = merge_callerids(callerids)

        # setup predefined resource information
        self._observation_info.observation = get_valid_states(mean_merged, deviation_merged, sample_size_merged)
        self._observation_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._observation_info


class TimestampSubs():
    """
    This class is used to subscribe to a topic, process the information for each callerid. It has a list
    which contains objects for each callerid that publish at this topic.
    """
    def __init__(self, config, use_global_subscriber):
        """
        Constructor to create a object of TimeoutSubs that exists per topic. The last callerid-config that
        defines no callerid exactly is used as default config for new callerids that are not explicitly
        named in the config. If there is no callerid-config that can be used as default config, unknown
        callerids are ignored.
        :param config: Configuration from yaml file
        """
        # topic that should be subscribed
        self.topic = Config.get_param(config, 'name')

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
                self._merged_bases.append(TimestampMergedBase(self.topic, callerid_config))
                if not len(callerid_list):
                    self._config_for_unknown = callerid_config
            except KeyError as e:
                rospy.logerr(e)

        # get topic class and subscribe
        if not use_global_subscriber:
            msg_class, real_topic, msg_eval = rostopic.get_topic_class(self.topic)
            self.sub = rospy.Subscriber(self.topic, msg_class, self.cb, queue_size=1)

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

        # print best_config

        if best_config:
            new_base = TimestampBase(self.topic, callerid, best_config)

        self._bases[callerid] = new_base
        return new_base

    def cb(self, msg):
        """
        Callback method that is called by the subscriber.
        After identifying the callerid the corresponding callback is called if the msg includes a header.
        If the callerid does not exists, it will be created as long as a suitable config can be found.
        If there exists a config with undefined callerids, this will be the default config.
        Otherwise the callerid is added to list, but without calling a callback.
        :param msg: message from publisher
        """

        curr_rostime = rospy.get_rostime().to_nsec()

        if not msg._has_header:
            # print 'no header!'
            return

        current_callerid = msg._connection_header['callerid']

        self._bases_lock.acquire()
        if current_callerid not in self._bases:
            base = self.add_callerid(current_callerid)
        else:
            base = self._bases.get(current_callerid)

        if base:
            base.cb(msg, curr_rostime)
        self._bases_lock.release()

    def get_observation_info(self):
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
            info += [merge.get_observation_info(callerids)]

        return info

    # def cleanup_dead_callerids(self):
    #     """
    #     Remove callerids from list if the number of timeouts reaches a maximum, but
    #     only if its not exactly defined in the config.
    #     """
    #     self._bases_lock.acquire()
    #
    #     callerids = list(self._bases.iterkeys())
    #
    #     for callerid in callerids:
    #         base = self._bases[callerid]
    #         if base and base.remaining_timeouts is not None and base.remaining_timeouts < 1:
    #             self._bases[callerid].stop()
    #             del self._bases[callerid]
    #
    #     self._bases_lock.release()


class Timestamp(PluginBase, PluginThread):
    """
    Timestamp main class. All necessary subscribers for the given topics are created.
    A additional thread is also started to publish the observer-info with a defined rate.
    """
    def __init__(self):
        """
        Constructor for Timeout main
        """
        PluginBase.__init__(self, "timestamp")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        self.rate = rospy.Rate(1)

    def run(self):
        """
        Thread runs in here, till shutdown. It will publish observer info with a defined
        rate and also cleanup old/unused callerids of topics.
        """

        while not rospy.is_shutdown():

            observation_data = []
            for sub in self.subs:
                observation_data += sub.get_observation_info()

            msg = observer_info(observation_infos=observation_data)
            self.info_pub.publish(msg)
            try:
                self.rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as e:
                print e

    def initialize(self, config):
        """
        Setup depending on the given config. For each topic a 'TimestampSubs'-object is created.
        In addition it start the main thread of this plugin.
        :param config: Configuration from yaml file
        """

        self.rate = rospy.Rate(Config.get_param(config, 'main_loop_rate'))
        try:
            use_global_subscriber = Config.get_param(config, 'use_global_subscriber')
        except KeyError:
            use_global_subscriber = False

        for topic_config in Config.get_param(config, 'topics'):
            try:
                self.subs.append(TimestampSubs(topic_config, use_global_subscriber))
            except (KeyError, StandardError) as e:
                rospy.logerr(e)

        sub_dict = dict()
        if use_global_subscriber:
            for sub in self.subs:
                # print sub
                sub_dict[sub.topic] = sub.cb

        self.start()

        return sub_dict


timestamp = Timestamp
