#!/usr/bin/env python
import rospy
from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration
import Queue

import unittest

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


class DiagnosisConfigValidator():
    # define which test is necessary to run the diagnosis
    CHECK_PUBLICATION_NECESSARY = True
    CHECK_SUBSCRIPTION_NECESSARY = False
    CHECK_OBSERVATION_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY = True
    CHECK_OBSERVATION_WITHOUT_PUBLICATION_NECESSARY = True
    CHECK_OBSERVATION_OF_PUBLISHED_TOPICS_NECESSARY = True
    CHECK_NAMING_OF_TOPICS_NECESSARY = True
    CHECK_TOPIC_LOOPS_OF_NODES_NECESSARY = False
    CHECK_NODES_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY = True
    CHECK_OBSERVED_NODES_THAT_DO_NOT_EXIST_NECESSARY = True
    CHECK_CHECK_UNIQUENESS_OF_NODE_NAMES = True

    DEBUG_LEVEL_DISABLED = 0
    DEBUG_LEVEL_RESULT = 1
    DEBUG_LEVEL_TESTS = 2
    DEBUG_LEVEL_VERBOSE = 3

    OBS_WITH_TOPICS = ['hz', 'timestamp', 'timeout', 'timing', "scores", 'velocity', 'movement']

    class NodesConfig(object):
        __slots__ = ['nodes', 'topics_published_from_nodes', 'topics_subscribed_from_nodes',
                     'nodes_publish_topics', 'nodes_subscribe_topics', 'topics_from_nodes']

        def __init__(self, nodes, topics_published_from_nodes, topics_subscribed_from_nodes,
                     nodes_publish_topics, nodes_subscribe_topics, topics_from_nodes):
            self.nodes = nodes
            self.topics_published_from_nodes = topics_published_from_nodes
            self.topics_subscribed_from_nodes = topics_subscribed_from_nodes
            self.nodes_publish_topics = nodes_publish_topics
            self.nodes_subscribe_topics = nodes_subscribe_topics
            self.topics_from_nodes = topics_from_nodes

    class ObserversConfig(object):
        __slots__ = ['topics', 'observed_resources']

        def __init__(self, topics, observed_resources):
            self.topics = topics
            self.observed_resources = observed_resources

    def __init__(self, config, debug=DEBUG_LEVEL_DISABLED):
        self.tests = [self.check_publication,
                      self.check_subscription,
                      self.check_observation_without_subscription_or_publication,
                      self.check_observation_without_publication,
                      self.check_observation_of_published_topics,
                      self.check_naming_of_topics,
                      self.check_topic_loops_of_nodes,
                      self.check_nodes_without_subscription_or_publication,
                      self.check_observed_nodes_that_do_not_exist,
                      self.check_uniqueness_of_node_names]

        self.debug = debug
        self.config = config

        # read nodes configuration
        nodes_config = self.init_nodes(self.config)
        if nodes_config:
            self.nodes = nodes_config.nodes
            self.topics_published_from_nodes = nodes_config.topics_published_from_nodes
            self.topics_subscribed_from_nodes = nodes_config.topics_subscribed_from_nodes
            self.nodes_publish_topics = nodes_config.nodes_publish_topics
            self.nodes_subscribe_topics = nodes_config.nodes_subscribe_topics
            self.topics_from_nodes = nodes_config.topics_from_nodes

        # read observer configuration
        observer_config = self.init_observers(self.config)
        if observer_config:
            self.topics = observer_config.topics
            self.observed_resources = observer_config.observed_resources

        if self.debug >= self.DEBUG_LEVEL_VERBOSE:
            print WARNING + 'Topics that are observerd by diagnosis config: %3d' % (len(self.topics)) + ENDC
            print WARNING + 'Topics that are used in config: %3d' % (len(self.topics_from_nodes | self.topics)) + ENDC

    @staticmethod
    def init_nodes(config):
        if not hasattr(config, 'nodes'):
            return None

        nodes = []
        topics_published_from_nodes = {}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {}
        nodes_subscribe_topics = {}
        topics_from_nodes = set()

        for node in config.nodes:
            node_name = node.name
            nodes.append(node_name)

            for topic in node.pub_topic:
                topics_published_from_nodes.setdefault(topic, []).append(node_name)
                nodes_publish_topics.setdefault(node_name, []).append(topic)

            for topic in node.sub_topic:
                topics_subscribed_from_nodes.setdefault(topic, []).append(node_name)
                nodes_subscribe_topics.setdefault(node_name, []).append(topic)

            topics_from_nodes.update(node.pub_topic)
            topics_from_nodes.update(node.sub_topic)

        return DiagnosisConfigValidator.NodesConfig(nodes=nodes,
                                                    topics_published_from_nodes=topics_published_from_nodes,
                                                    topics_subscribed_from_nodes=topics_subscribed_from_nodes,
                                                    nodes_publish_topics=nodes_publish_topics,
                                                    nodes_subscribe_topics=nodes_subscribe_topics,
                                                    topics_from_nodes=topics_from_nodes)

    @staticmethod
    def init_observers(config):
        if not hasattr(config, 'observers'):
            return None

        topics = set()
        observed_resources = set()
        for obs in config.observers:

            if obs.type in DiagnosisConfigValidator.OBS_WITH_TOPICS:
                [observed_resources.update([('topic', res)]) for res in obs.resource]
                [topics.update([item] if isinstance(item, str) else item) for item in obs.resource]
            else:
                [observed_resources.update([('node', res)]) for res in obs.resource]

        return DiagnosisConfigValidator.ObserversConfig(topics=topics, observed_resources=observed_resources)

    def check_publication(self):
        published_topics = [topic for topic in self.topics_from_nodes if
                            topic in self.topics_published_from_nodes.keys()]
        not_published_topics = [topic for topic in self.topics_from_nodes if
                                topic not in self.topics_published_from_nodes.keys()]

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(not_published_topics):
            print WARNING + 'Topics that are published / not published: %3d / %3d' % (
                len(published_topics), len(not_published_topics)) + ENDC
            print "not published topics:"
            print not_published_topics

        return True if not len(not_published_topics) or not self.CHECK_PUBLICATION_NECESSARY else False

    def check_subscription(self):
        subscribed_topics = [topic for topic in self.topics_from_nodes if
                             topic in self.topics_subscribed_from_nodes.keys()]
        not_subscribed_topics = [topic for topic in self.topics_from_nodes if
                                 topic not in self.topics_subscribed_from_nodes.keys()]

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(not_subscribed_topics):
            print WARNING + 'Topics that are subscribed / not subscribed: %3d / %3d' % (
                len(subscribed_topics), len(not_subscribed_topics)) + ENDC
            print "not subscribed topics:"
            print not_subscribed_topics

        return True if not len(not_subscribed_topics) or not self.CHECK_SUBSCRIPTION_NECESSARY else False

    def check_observation_without_subscription_or_publication(self):
        ghost_topics = [topic for topic in self.topics if topic not in self.topics_from_nodes]

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(ghost_topics):
            print WARNING + 'Topics observed but no node subscribes or publishs: %3d' % (len(ghost_topics)) + ENDC
            print ghost_topics

        if not len(ghost_topics) or not self.CHECK_OBSERVATION_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY:
            return True
        return False

    def check_observation_without_publication(self):
        published_topics = [topic for topic in self.topics_from_nodes if
                            topic in self.topics_published_from_nodes.keys()]
        publishless_topics = [topic for topic in self.topics if topic not in published_topics]

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(publishless_topics):
            print WARNING + 'Topics observed but without publishing node: %3d' % (len(publishless_topics)) + ENDC
            print publishless_topics

        if not len(publishless_topics) or not self.CHECK_OBSERVATION_WITHOUT_PUBLICATION_NECESSARY:
            return True
        return False

    def check_observation_of_published_topics(self):
        useless_topics = [topic for topic in self.topics_from_nodes if
                          topic not in self.topics and topic not in self.topics_subscribed_from_nodes.keys()]

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(useless_topics):
            print WARNING + 'Topics published by nodes but not observed or subscribed: %3d' % (
                len(useless_topics)) + ENDC
            print useless_topics

        if not len(useless_topics) or not self.CHECK_OBSERVATION_OF_PUBLISHED_TOPICS_NECESSARY:
            return True
        return False

    def check_naming_of_topics(self):
        all_topics = self.topics_from_nodes | self.topics
        forbidden_topics = set()
        for entry in all_topics:
            if any(x in entry for x in ['$', '#', '|', ' ', '-', 'AB', '/AB']):
                forbidden_topics.update([entry])

        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(forbidden_topics):
            print WARNING + 'Topics that contains forbidden characters: %3d' % (len(forbidden_topics)) + ENDC
            print list(forbidden_topics)

        if not len(forbidden_topics) or not self.CHECK_NAMING_OF_TOPICS_NECESSARY:
            return True
        return False

    def check_topic_loops_of_nodes(self):
        loop_found = False
        for node in self.config.nodes:
            node_name = node.name
            topic_in_loop = set(node.pub_topic) & set(node.sub_topic)
            if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(topic_in_loop):
                loop_found |= True
                print WARNING + 'Topics that are published and subscribed by the same node %s: %3d' % (
                    node_name, (len(topic_in_loop))) + ENDC
                print list(topic_in_loop)

        if not loop_found or not self.CHECK_TOPIC_LOOPS_OF_NODES_NECESSARY:
            return True
        return False

    def check_nodes_without_subscription_or_publication(self):
        nodes_without_subscription_or_publication = [node.name for node in self.config.nodes if
                                                     not len(node.pub_topic) and not len(node.sub_topic)]

        observed_nodes = []

        [observed_nodes.extend(obs.resource) for obs in self.config.observers if obs.type not in self.OBS_WITH_TOPICS]

        useless_nodes = set(nodes_without_subscription_or_publication) - set(observed_nodes)
        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(useless_nodes):
            print WARNING + 'Nodes without subscription, publication and observation: %3d' % (len(useless_nodes)) + ENDC
            print list(useless_nodes)

        if not len(useless_nodes) or not self.CHECK_NODES_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY:
            return True
        return False

    def check_observed_nodes_that_do_not_exist(self):
        nodes = [node.name for node in self.config.nodes]

        observed_nodes = []

        [observed_nodes.extend(obs.resource) for obs in self.config.observers if obs.type not in self.OBS_WITH_TOPICS]

        ghost_nodes = set(observed_nodes) - set(nodes)
        if self.debug >= self.DEBUG_LEVEL_VERBOSE and len(ghost_nodes):
            print WARNING + 'Observed nodes that do not exist in nodes config: %3d' % (len(ghost_nodes)) + ENDC
            print list(ghost_nodes)

        if not len(ghost_nodes) or not self.CHECK_OBSERVED_NODES_THAT_DO_NOT_EXIST_NECESSARY:
            return True
        return False

    def check_uniqueness_of_node_names(self):
        nodes = [node.name for node in self.config.nodes]
        num_nodes = len(nodes)
        num_unique_nodes = len(set(nodes))
        if self.debug >= self.DEBUG_LEVEL_VERBOSE and num_nodes is not num_unique_nodes:
            print WARNING + 'Some nodes are not unique!' + ENDC
            [nodes.remove(name) for name in set(nodes) if name in nodes]
            print list(nodes)

        if num_nodes is num_unique_nodes or not self.CHECK_CHECK_UNIQUENESS_OF_NODE_NAMES:
            return True
        return False

    def run_tests(self):
        result = True

        for test in self.tests:
            if self.debug >= self.DEBUG_LEVEL_TESTS:
                print HEADER + 'running', test.__name__ + ENDC
            try:
                passed = test()
            except AttributeError:
                return False
            result &= passed
            if self.debug >= self.DEBUG_LEVEL_TESTS:
                print OKGREEN + 'passed' + ENDC if passed else FAIL + 'failed' + ENDC

        if self.debug >= self.DEBUG_LEVEL_RESULT:
            print OKGREEN + 'all config tests passed' + ENDC if result else FAIL + 'some config tests failed' + ENDC
        return result

    @staticmethod
    def minimize_config(config):
        # read nodes config
        nodes_config = DiagnosisConfigValidator.init_nodes(config)
        # read observer config
        observer_config = DiagnosisConfigValidator.init_observers(config)

        if not nodes_config or not observer_config:
            return

        # get unused nodes and topics
        all_nodes_and_topics = set([('node', node) for node in nodes_config.nodes])
        all_nodes_and_topics |= set([('topic', x) for x in nodes_config.topics_from_nodes])

        queue = Queue.LifoQueue()
        for entry in observer_config.observed_resources:
            queue.put_nowait(entry)

        while True:
            try:
                item = queue.get_nowait()
                if item not in all_nodes_and_topics:
                    continue

                all_nodes_and_topics.remove(item)
                if item[0] == 'node':
                    [queue.put_nowait(('topic', entry)) for entry in nodes_config.nodes_subscribe_topics[item[1]]]
                else:
                    [queue.put_nowait(('node', entry)) for entry in nodes_config.topics_published_from_nodes[item[1]]]
            except Queue.Empty:
                break
            except KeyError:
                continue

        # clean nodes
        unobserved_topics = [resource for (resource_type, resource) in all_nodes_and_topics if resource_type == 'topic']
        unobserved_nodes = [resource for (resource_type, resource) in all_nodes_and_topics if resource_type == 'node']

        for resource in unobserved_nodes:
            for index in xrange(len(config.nodes) - 1, -1, -1):
                node = config.nodes[index]
                if resource == node.name and set(node.pub_topic).issubset(unobserved_topics):
                    del config.nodes[index]

        for node in config.nodes:
            for topic in unobserved_topics:
                if topic in node.sub_topic:
                    node.sub_topic.remove(topic)
                if topic in node.pub_topic:
                    node.pub_topic.remove(topic)

        # clean observers
        unconfigured_topics = [resource for (resource_type, resource) in observer_config.observed_resources if
                               resource_type == 'topic' and
                               resource not in nodes_config.topics_published_from_nodes.keys()]

        unconfigured_nodes = [resource for (resource_type, resource) in observer_config.observed_resources if
                              resource_type == 'node' and
                              resource not in nodes_config.nodes]

        for index in xrange(len(config.observers) - 1, -1, -1):
            if config.observers[index].type in DiagnosisConfigValidator.OBS_WITH_TOPICS:
                if set(config.observers[index].resource).issubset(set(unconfigured_topics)):
                    del config.observers[index]
            else:
                if set(config.observers[index].resource).issubset(set(unconfigured_nodes)):
                    del config.observers[index]


def check_list_in_list(compare_fct, first, second, enable_equal_check=False):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # find each element of the first list in the second list
    for entry_of_first in first:
        first_found_in_second = False
        for entry_of_second in second:
            if compare_fct(entry_of_first, entry_of_second):
                first_found_in_second = True
                break
        if not first_found_in_second:
            return False

    if enable_equal_check:
        return check_list_in_list(compare_fct, second, first)
    return True


def compare_node_configuration(first, second):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # it's not allowed, if resources, mode_msg, or mode are different
    if not first.name == second.name:
        return False
    if not check_list_in_list(lambda a, b: a == b, first.sub_topic, second.sub_topic, True):
        return False
    if not check_list_in_list(lambda a, b: a == b, first.pub_topic, second.pub_topic, True):
        return False
    return True


def compare_observer_configuration(first, second):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # it's not allowed, if resources, mode_msg, or mode are different
    if not first.type == second.type:
        return False
    if not check_list_in_list(lambda a, b: a == b, first.resource, second.resource, True):
        return False
    return True


def compare_configuration(first, second):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # find each element of the first list in the second list and each element of the second list in the first list
    if not check_list_in_list(compare_node_configuration, first.nodes, second.nodes, True):
        return False
    if not check_list_in_list(compare_observer_configuration, second.observers, first.observers, True):
        return False

    return True

if __name__ == "__main__":
    rospy.init_node('tug_diagnosis_config_validator', anonymous=False)

    # config_a = configuration()
    # # config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
    # # config_a.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=[]))
    # # config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
    # # config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic1", "/topic2"]))
    # # config_a.nodes.append(node_configuration(name="node3", pub_topic=[], sub_topic=[]))
    # # config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
    # # config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
    # # config_a.observers.append(observer_configuration(type="resource", resource=["node3"]))
    #
    a_config = configuration()
    a_config.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
    a_config.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
    a_config.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
    # config_a.observers.append(observer_configuration(type="movement", resource=["/topic1", "/topic2"]))

    a_config.observers.append(observer_configuration(type="resource", resource=["node1"]))
    a_config.observers.append(observer_configuration(type="resource", resource=["node2"]))
    a_config.observers.append(observer_configuration(type="resource", resource=["node3"]))

    # config_b = configuration()
    # config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
    # config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
    # config_b.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))

    # config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
    # config_b.observers.append(observer_configuration(type="resource", resource=["node2"]))
    # config_b.observers.append(observer_configuration(type="resource", resource=["node3"]))

    # print compare_configuration(config_a, config_b)

    validator = DiagnosisConfigValidator(a_config, debug=DiagnosisConfigValidator.DEBUG_LEVEL_VERBOSE)
    # print validator.run_tests()
    validator.minimize_config(a_config)

    print validator.config


class TestDiagnosisConfigValidator(unittest.TestCase):
    def setUp(self):
        pass

    def test_compare_configuration_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        self.assertTrue(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["WRONG"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_3(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz"))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_4(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="Wrong", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_5(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_6(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["Wrong"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_7(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_8(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["Wrong"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_9(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_10(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="Wrong", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_11(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_12(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2", "node1"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_13(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1", "node2"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_14(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1", "/topic2"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_compare_configuration_15(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["node2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2", "/topic3"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["node2"]))

        self.assertFalse(compare_configuration(config_a, config_b), "configurations not equal!")

    def test_minimize_config_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node2"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node3"]))

        config_required = configuration()
        config_required.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_required.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_required.nodes.append(node_configuration(name="node3", pub_topic=[], sub_topic=["/topic2"]))
        config_required.observers.append(observer_configuration(type="resource", resource=["node1"]))
        config_required.observers.append(observer_configuration(type="resource", resource=["node2"]))
        config_required.observers.append(observer_configuration(type="resource", resource=["node3"]))

        DiagnosisConfigValidator.minimize_config(config_a)

        self.assertTrue(compare_configuration(config_a, config_required), "configurations not equal!")

    def test_minimize_config_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
        config_a.observers.append(observer_configuration(type="resource", resource=["node1"]))

        config_required = configuration()
        config_required.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=[]))
        config_required.observers.append(observer_configuration(type="resource", resource=["node1"]))

        DiagnosisConfigValidator.minimize_config(config_a)

        self.assertTrue(compare_configuration(config_a, config_required), "configurations not equal!")
