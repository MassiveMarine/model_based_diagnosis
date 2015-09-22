#!/usr/bin/env python

from threading import Lock

import rospy

from tug_observers import PluginBase, PluginThread, PluginTimeout
from tug_observers_msgs.msg import observer_error, resource_error
from tug_python_utils import YamlHelper as Config


error_pub = None

# predefined resource error msgs that are used if a error is published
resource_error_timeout = resource_error(error_msg='Timeout',
                                        verbose_error_msg='Timeout of Topic',
                                        error=resource_error.NO_AVAILABLE)


class TimeoutBase():
    """
    This class is used for each callerid. Each instance of this has its own timeout thread.
    """
    def __init__(self, topic, callerid, config):
        """
        Constructor of the TimeoutBase class. It creates and starts the timeout-thread.
        :param topic: name of topic that is subscribed
        :param callerid: name of the node that publish on the topic
        :param config: Configuration from yaml file
        """
        # requirements to handle timeout
        self._event = PluginTimeout(Config.get_param(config, 'timeout'), self.timeout_cb)
        self.max_timeouts = Config.get_param(config, 'max_timeouts_in_a_row') if len(Config.get_param(config, 'callerid')) < 1 else None
        self.remaining_timeouts = self.max_timeouts

        # create a predefined error msg
        self._observer_error = observer_error(type='timeout', resource=str(topic + '[' + str(callerid)) + ']')

        # print self.max_timeouts, Config.get_param(config, 'timeout')

    def timeout_cb(self):
        """
        Callback method that is called if a timeout is reached.
        It will publish a error and decrements the remaining timeouts counter.
        """
        # decrement timeout counter
        if self.max_timeouts is not None:
            self.remaining_timeouts -= 1

        if not error_pub:
            return

        # publish error
        if self.remaining_timeouts is None or self.remaining_timeouts >= 0:
            # print 'self.remaining_timeouts: ', self.remaining_timeouts
            self._observer_error.header = rospy.Header(stamp=rospy.Time.now())
            self._observer_error.error_msg = resource_error_timeout
            error_pub.publish(self._observer_error)

    def cb(self, msg):
        """
        Callback method that is called from the forwarding of TimeoutSub.
        Calculates the delay between this and the previous callback call and updates the filter.
        :param msg: message from publisher
        """
        self._event.set()
        self.remaining_timeouts = self.max_timeouts

    def stop(self):
        """
        Stop the observation of this callerid. This will stop the Timeout thread.
        """
        self._event.stop()


class TimeoutSubs():
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
        self._config_for_unknown = None

        # init all states per callerid-combination and detect a possible
        # config that can be used as default config.
        for callerid_config in self._callerids_config:
            try:
                callerid_list = Config.get_param(callerid_config, 'callerid')
                if not len(callerid_list):
                    self._config_for_unknown = callerid_config
            except KeyError as e:
                rospy.logerr(e)

        # subscribe to given topic
        if not use_global_subscriber:
            self.sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.cb, queue_size=1)

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
            new_base = TimeoutBase(self.topic, callerid, best_config)

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
        # rospy.loginfo(current_callerid)

        self._bases_lock.acquire()
        if current_callerid not in self._bases:
            base = self.add_callerid(current_callerid)
        else:
            base = self._bases.get(current_callerid)

        if base:
            base.cb(msg)
        self._bases_lock.release()

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


class Timeout(PluginBase, PluginThread):
    """
    Timeout main class. All necessary subscribers for the given topics are created.
    A additional thread is also started to publish the observer-info with a defined rate.
    """
    def __init__(self):
        """
        Constructor for Timeout main
        """
        PluginBase.__init__(self, "timeout")
        PluginThread.__init__(self)

        self.subs = []
        self.topics = None
        global error_pub
        error_pub = self.error_pub

    def run(self):
        """
        Thread runs in here, till shutdown. It will publish observer info with a defined
        rate and also cleanup old/unused callerids of topics.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for sub in self.subs:
                sub.cleanup_dead_callerids()

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as e:
                print e

    def initialize(self, config):
        """
        Setup depending on the given config. For each topic a 'HzSubs'-object is created.
        In addition it start the main thread of this plugin.
        :param config: Configuration from yaml file
        """

        try:
            use_global_subscriber = Config.get_param(config, 'use_global_subscriber')
        except KeyError:
            use_global_subscriber = False

        for topic_config in Config.get_param(config, 'topics'):
            try:
                self.subs.append(TimeoutSubs(topic_config, use_global_subscriber))
            except (KeyError, StandardError) as e:
                rospy.logerr(e)

        sub_dict = dict()
        for sub in self.subs:
            sub_dict[sub.topic] = sub.cb

        self.start()

        return sub_dict


timeout = Timeout