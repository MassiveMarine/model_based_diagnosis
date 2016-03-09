#!/usr/bin/env python
import rospy

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


class DiagnosisConfigValidater():
    # define which test is necessary to run the diagnosis
    CHECK_PUBLICATION_NECESSARY = True
    CHECK_SUBSCRIPTION_NECESSARY = False
    CHECK_OBSERVATION_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY = True
    CHECK_OBSERVATION_WITHOUT_PUBLICATION_NECESSARY = True
    CHECK_OBSERVATION_OF_PUBLISHED_OR_SUBSCRIBED_TOPICS_NECESSARY = True
    CHECK_NAMING_OF_TOPICS_NECESSARY = True
    CHECK_TOPIC_LOOPS_OF_NODES_NECESSARY = True

    def __init__(self, configs, debug=False):
        self.tests = [self.check_publication,
                      self.check_subscription,
                      self.check_observation_without_subscription_or_publication,
                      self.check_observation_without_publication,
                      self.check_observation_of_published_or_subscribed_topics,
                      self.check_naming_of_topics,
                      self.check_topic_loops_of_nodes, ]

        self.debug = debug
        self.configs = configs
        self.nodes = []
        self.topics_published_from_nodes = {}
        self.topics_subscribed_from_nodes = {}
        self.nodes_publish_topics = {}
        self.nodes_subscribe_topics = {}

        self.topics_from_nodes = set()

        # read nodes configuration
        for node in configs['nodes']:
            node_name = node['name']
            self.nodes.append(node_name)

            for topic in node.get('pub_topic', []):
                self.topics_published_from_nodes.setdefault(topic, []).append(node_name)
                self.nodes_publish_topics.setdefault(node_name, []).append(topic)

            for topic in node.get('sub_topic', []):
                self.topics_subscribed_from_nodes.setdefault(topic, []).append(node_name)
                self.nodes_subscribe_topics.setdefault(node_name, []).append(topic)

            self.topics_from_nodes.update(node.get('pub_topic', []))
            self.topics_from_nodes.update(node.get('sub_topic', []))

        # read observation configuration
        self.topics = set()
        for obs in configs['observations']:
            if 'topics' in obs:
                entry = obs['topics']

                for item in entry:
                    self.topics.update([item] if isinstance(item, str) else item)

        if self.debug:
            print WARNING + 'Topics that are observerd by diagnosis config: %3d' % (len(self.topics)) + ENDC
            print WARNING + 'Topics that are used in config: %3d' % (len(self.topics_from_nodes | self.topics)) + ENDC

    def check_publication(self):
        published_topics = [topic for topic in self.topics_from_nodes if
                            topic in self.topics_published_from_nodes.keys()]
        not_published_topics = [topic for topic in self.topics_from_nodes if
                                topic not in self.topics_published_from_nodes.keys()]

        if self.debug:
            print WARNING + 'Topics that are published / not published:   %3d / %3d' % (
                len(published_topics), len(not_published_topics)) + ENDC
            print "not published topics:"
            print not_published_topics

        return True if not len(not_published_topics) or not self.CHECK_PUBLICATION_NECESSARY else False

    def check_subscription(self):
        subscribed_topics = [topic for topic in self.topics_from_nodes if
                             topic in self.topics_subscribed_from_nodes.keys()]
        not_subscribed_topics = [topic for topic in self.topics_from_nodes if
                                 topic not in self.topics_subscribed_from_nodes.keys()]

        if self.debug:
            print WARNING + 'Topics that are subscribed / not subscribed: %3d / %3d' % (
                len(subscribed_topics), len(not_subscribed_topics)) + ENDC
            print "not subscribed topics:"
            print not_subscribed_topics

        return True if not len(not_subscribed_topics) or not self.CHECK_SUBSCRIPTION_NECESSARY else False

    def check_observation_without_subscription_or_publication(self):
        ghost_topics = [topic for topic in self.topics if topic not in self.topics_from_nodes]

        if self.debug:
            print WARNING + 'Topics observed but no node subscribes or publishs: %3d' % (len(ghost_topics)) + ENDC
            print ghost_topics

        return True if not len(
            ghost_topics) or not self.CHECK_OBSERVATION_WITHOUT_SUBSCRIPTION_OR_PUBLICATION_NECESSARY else False

    def check_observation_without_publication(self):
        published_topics = [topic for topic in self.topics_from_nodes if
                            topic in self.topics_published_from_nodes.keys()]
        publishless_topics = [topic for topic in self.topics if topic not in published_topics]

        if self.debug:
            print WARNING + 'Topics observed but without publishing node: %3d' % (len(publishless_topics)) + ENDC
            print publishless_topics

        return True if not len(
            publishless_topics) or not self.CHECK_OBSERVATION_WITHOUT_PUBLICATION_NECESSARY else False

    def check_observation_of_published_or_subscribed_topics(self):
        useless_topics = [topic for topic in self.topics_from_nodes if topic not in self.topics]

        if self.debug:
            print WARNING + 'Topics subscribed or published by nodes but not observed: %3d' % (
                len(useless_topics)) + ENDC
            print useless_topics

        return True if not len(
            useless_topics) or not self.CHECK_OBSERVATION_OF_PUBLISHED_OR_SUBSCRIBED_TOPICS_NECESSARY else False

    def check_naming_of_topics(self):
        all_topics = self.topics_from_nodes | self.topics
        forbidden_topics = set()
        for entry in all_topics:
            if any(x in entry for x in ['$', '#', '|', ' ', '-', 'AB', '/AB']):
                forbidden_topics.update([entry])

        if self.debug:
            print WARNING + 'Topics that contains forbidden characters: %3d' % (len(forbidden_topics)) + ENDC
            print list(forbidden_topics)

        return True if not len(forbidden_topics) or not self.CHECK_NAMING_OF_TOPICS_NECESSARY else False

    def check_topic_loops_of_nodes(self):
        loop_found = False
        for node in self.configs['nodes']:
            node_name = node['name']
            topic_in_loop = set(node.get('pub_topic', [])) & set(node.get('sub_topic', []))
            if self.debug and len(topic_in_loop):
                loop_found |= True
                print WARNING + 'Topics that are published and subscribed by the same node %s: %3d' % (
                    node_name, (len(topic_in_loop))) + ENDC
                print list(topic_in_loop)

        return True if not loop_found or not self.CHECK_TOPIC_LOOPS_OF_NODES_NECESSARY else False

    def run_tests(self):
        result = True

        for test in self.tests:
            print HEADER + 'running', test.__name__ + ENDC
            passed = test()
            result &= passed
            if self.debug:
                print OKGREEN + 'passed' + ENDC if passed else FAIL + 'failed' + ENDC

        return result


if __name__ == "__main__":
    rospy.init_node('tug_yaml_validater', anonymous=False)

    configs = rospy.get_param('/tug_diagnosis_node')

    print HEADER + 'init' + ENDC
    validater = DiagnosisConfigValidater(configs, debug=True)
    print OKGREEN + 'all passed' + ENDC if validater.run_tests() else FAIL + 'some failed' + ENDC

    exit(0)
