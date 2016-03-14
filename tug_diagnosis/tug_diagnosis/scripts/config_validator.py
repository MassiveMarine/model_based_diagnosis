#!/usr/bin/env python
import rospy
from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration
import Queue

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
    CHECK_TOPIC_LOOPS_OF_NODES_NECESSARY = True
    CHECK_NODES_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY = True
    CHECK_OBSERVED_NODES_THAT_DO_NOT_EXIST_NECESSARY = True

    DEBUG_LEVEL_DISABLED = 0
    DEBUG_LEVEL_RESULT = 1
    DEBUG_LEVEL_TESTS = 2
    DEBUG_LEVEL_VERBOSE = 3

    OBS_WITH_TOPICS = ['hz', 'timestamp', 'timeout', 'timing', "scores", 'velocity']

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
                      self.check_observed_nodes_that_do_not_exist]

        self.debug = debug
        self.config = config

        # read nodes configuration
        nodes_config = self.init_nodes(self.config)
        self.nodes = nodes_config.nodes
        self.topics_published_from_nodes = nodes_config.topics_published_from_nodes
        self.topics_subscribed_from_nodes = nodes_config.topics_subscribed_from_nodes
        self.nodes_publish_topics = nodes_config.nodes_publish_topics
        self.nodes_subscribe_topics = nodes_config.nodes_subscribe_topics
        self.topics_from_nodes = nodes_config.topics_from_nodes

        # read observer configuration
        observer_config = self.init_observers(self.config)
        self.topics = observer_config.topics
        self.observed_resources = observer_config.observed_resources

        if self.debug >= self.DEBUG_LEVEL_VERBOSE:
            print WARNING + 'Topics that are observerd by diagnosis config: %3d' % (len(self.topics)) + ENDC
            print WARNING + 'Topics that are used in config: %3d' % (len(self.topics_from_nodes | self.topics)) + ENDC

    @staticmethod
    def init_nodes(config):
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

    def run_tests(self):
        result = True

        for test in self.tests:
            if self.debug >= self.DEBUG_LEVEL_TESTS:
                print HEADER + 'running', test.__name__ + ENDC
            passed = test()
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

        unobserved_topics = [resource for (resource_type, resource) in all_nodes_and_topics if resource_type == 'topic']

        for (resource_type, resource) in all_nodes_and_topics:
            if resource_type == 'node':
                print resource
                for index, node in enumerate(config.nodes):
                    if resource == node.name and set(node.pub_topic).issubset(unobserved_topics):
                        del config.nodes[index]

        for node in config.nodes:
            for topic in unobserved_topics:
                if topic in node.sub_topic:
                    node.sub_topic.remove(topic)
                if topic in node.pub_topic:
                    node.pub_topic.remove(topic)


if __name__ == "__main__":
    rospy.init_node('tug_diagnosis_config_validator', anonymous=False)

    configA = configuration()
    configA.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
    configA.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
    configA.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic1", "/topic2"]))
    # configA.nodes.append(node_configuration(name="node3", pub_topic=[], sub_topic=[]))
    # configA.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
    configA.observers.append(observer_configuration(type="resource", resource=["node3"]))

    validator = DiagnosisConfigValidator(configA, debug=DiagnosisConfigValidator.DEBUG_LEVEL_VERBOSE)
    print validator.run_tests()
    validator.minimize_config(configA)

    exit(0)
