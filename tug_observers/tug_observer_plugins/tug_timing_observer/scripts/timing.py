#!/usr/bin/env python

from threading import Lock

import rospy

from tug_observers import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import observer_error, observer_info, resource_info, resource_error
from tug_observer_plugin_utils import Filter, SingleValueHypothesisCheckFactory
from tug_python_utils import YamlHelper as Config


error_pub = None

# predefined resource error msgs that are used if a error is published
resource_error_timeout = resource_error(error_msg='Timeout',
                                        verbose_error_msg='Timeout of Topic',
                                        error=resource_error.NOT_AVAILABLE)
resource_error_no_state_fits = resource_error(error_msg='No State fits',
                                              verbose_error_msg='No state can be found for the measured results',
                                              error=resource_error.NO_STATE_FITS)


class TimingState():
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
        frequency = Config.get_param(config, 'delay')
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


class TimingBase():
    """
    This class is used to calculate the delay between two topics. After a topic
    of b the next received topic of a is the start time. The next topic of b is
    the end point and the difference is calculated.
    """
    def __init__(self, config):
        """
        Constructor of the TimingBase class. It manages the filter, creates and
        setups the states and initialize stuff for the delay calculation.
        :param config: Configuration from yaml file
        """
        # necessary stuff to calculate time between topics
        self._first_topic_time_a = None
        self._first_topic_time_a_lock = Lock()

        # topics and callerids
        topicA = Config.get_param(config, 'topicA')
        topicB = Config.get_param(config, 'topicB')

        calleridA = Config.get_param(config, 'calleridA')
        calleridB = Config.get_param(config, 'calleridB')

        # Try to load and setup all states as defined in the yaml-file
        self._states = []
        for state_config in Config.get_param(config, 'states'):
            try:
                self._states.append(TimingState(state_config))
            except KeyError as e:
                rospy.logerr(e)

        # create a predefined msgs for info and error
        self._resource_info = resource_info(type='timing', resource=str(topicA + str(calleridA) + ' --> ' + topicB + str(calleridB)))
        self._observer_error = observer_error(type='timing', resource=str(topicA + str(calleridA) + ' --> ' + topicB + str(calleridB)))

        # create a filter
        self._filter = Filter(Config.get_param(config, 'filter'))

    def set_topic_a(self, time):
        """
        Set time for the current callback of the first topic. It is
        set only if it is required by topic b and it is currently not set.
        :param time: rostime at which the msg was received
        """
        self._first_topic_time_a_lock.acquire()
        if self._first_topic_time_a is 0:
            self._first_topic_time_a = time

        self._first_topic_time_a_lock.release()

    def set_topic_b(self, time):
        """
        Set time for the current callback of the second topic. It is
        set and enable the first the storage of the first topic.
        :param time: rostime at which the msg was received
        """
        self._first_topic_time_a_lock.acquire()
        if self._first_topic_time_a:
            delay = time - self._first_topic_time_a
            self._filter.update(delay)
            print delay

        self._first_topic_time_a = 0
        self._first_topic_time_a_lock.release()

    def get_resource_info(self):
        """
        Generate a resource information for publishing by readout the filter and
        make hypotheses checks.
        :return a resource information that can be published
        """
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

        mean, deviation, sample_size = self._filter.get_value()

        # check if filter contains usable information
        if mean is None:
            self._resource_info.states = []
            self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
            return self._resource_info

        # setup predefined resource information
        self._resource_info.states = get_valid_states(mean, deviation, sample_size)
        self._resource_info.header = rospy.Header(stamp=rospy.Time.now())
        return self._resource_info


class TimingTopicGroup():
    """
    This class is used to subscribe to the topics and filter the callerids
    if defined and manage the callback methods.
    """
    def __init__(self, config):
        """
        Constructor to create a object of TimingTopicGroup that exists per topic-pair.
        The callback methods are defined depending on callerid filters.
        :param config: Configuration from yaml file
        """
        # topics that should be subscribed
        self._topicA = Config.get_param(config, 'topicA')
        self._topicB = Config.get_param(config, 'topicB')

        self._callerA = Config.get_param(config, 'calleridA')
        self._callerB = Config.get_param(config, 'calleridB')

        self._base = TimingBase(config)

        # callbacks and subscriptions to given topics
        self._cb_a = self._cb_a_with_callerid if len(self._callerA) else self._cb_a_without_callerid
        self._cb_b = self._cb_b_with_callerid if len(self._callerB) else self._cb_b_without_callerid

        self.sub_a = rospy.Subscriber(self._topicA, rospy.AnyMsg, self._cb_a, queue_size=1)
        self.sub_b = rospy.Subscriber(self._topicB, rospy.AnyMsg, self._cb_b, queue_size=1)

    def _cb_a_with_callerid(self, msg):
        """
        Callback method that is called by the subscriber of topic a.
        Check if callerid is in list and forward to basic cb.
        :param msg: message from publisher
        """
        current_callerid = msg._connection_header['callerid']
        if current_callerid in self._callerA:
            self._cb_a_without_callerid(msg)

    def _cb_a_without_callerid(self, msg):
        """
        Callback method that is called by the subscriber of topic a.
        :param msg: message from publisher
        """
        self._base.set_topic_a(rospy.get_rostime().to_sec())

    def _cb_b_with_callerid(self, msg):
        """
        Callback method that is called by the subscriber of topic b.
        Check if callerid is in list and forward to basic cb.
        :param msg: message from publisher
        """
        current_callerid = msg._connection_header['callerid']
        if current_callerid in self._callerA:
            self._cb_b_without_callerid(msg)

    def _cb_b_without_callerid(self, msg):
        """
        Callback method that is called by the subscriber of topic b.
        :param msg: message from publisher
        """
        self._base.set_topic_b(rospy.get_rostime().to_sec())

    def get_resource_info(self):
        """
        Create array of resource-info of all defined combinations of callerids.
        """
        return [self._base.get_resource_info()]


class Timing(PluginBase, PluginThread):
    """
    Delay main class. All necessary subscribers for the given topics are created.
    A additional thread is also started to publish the observer-info with a defined rate.
    """
    def __init__(self):
        """
        Constructor for Delay main
        """
        PluginBase.__init__(self, "timing")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        global error_pub
        error_pub = self.error_pub
        self.rate = rospy.Rate(1)

    def run(self):
        """
        Thread runs in here, till shutdown. It will publish observer info with a defined
        rate.
        """

        while not rospy.is_shutdown():
            resource_data = []
            for sub in self.subs:
                resource_data += sub.get_resource_info()

            msg = observer_info(resource_infos=resource_data)
            self.info_pub.publish(msg)
            self.rate.sleep()

    def initialize(self, config):
        """
        Setup depending on the given config. For each topic a 'HzSubs'-object is created.
        In addition it start the main thread of this plugin.
        :param config: Configuration from yaml file
        """
        self.rate = rospy.Rate(Config.get_param(config, 'main_loop_rate'))
        for topic_config in Config.get_param(config, 'topics'):
            try:
                self.subs.append(TimingTopicGroup(topic_config))
            except (KeyError, StandardError) as e:
                rospy.logerr(e)

        self.start()

timing = Timing