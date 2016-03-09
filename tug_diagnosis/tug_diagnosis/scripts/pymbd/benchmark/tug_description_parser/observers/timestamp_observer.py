from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *
from tug_diagnosis_msgs.msg import observer_configuration
import unittest


class TimestampObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, ab_node, observation, ab_subscribed_topics):
        BaseObserver.__init__(self)
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(observation)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)

        self.ab_node = ab_node
        self.observation = observation
        self.ab_subscribed_topics = ab_subscribed_topics

    def __repr__(self):
        return "timestamp: %s, %s, %s" % (self.ab_node, self.observation, self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos([self.ab_node] + self.ab_subscribed_topics) + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        config_type = str('timestamp')
        if not isinstance(config, observer_configuration):
            raise TypeError
        if not config.type == config_type:
            raise KeyError('given type in config is wrong!')

        # checkInputData.dict_data_valid(config, False)
        # topics = config['topics']

        topics = config.resource

        checkInputData.list_data_valid(topics, num_entries=1)

        # checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False, allow_empty=False)
        # checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=True, entry_type=list,
                                       allow_empty=False)
        checkInputData.list_data_valid(topics_published_from_nodes.keys(), check_entries=True)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=True, entry_type=list, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        topic = topics[0]
        callerids = topics_published_from_nodes.get(topic, [])
        checkInputData.list_data_valid(callerids, allow_empty=False)

        for callerid in callerids:

            observation = config_type + "_obs_" + topic + "_" + callerid
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)

            subscribed_topics = nodes_subscribe_topics.get(callerid, [])
            rules.append(TimestampObserver(ab_pred(str(callerid)), observation, all_ab_pred(subscribed_topics)))

            if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
                raise ValueError('subscribed topics are not not in topics published list!')

        new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter(config_type, topic,
                                                                                    topics_published_from_nodes[topic])
        vars.update(new_vars)
        rules += new_rules
        nodes += new_nodes

        return vars, rules, nodes, []

    @staticmethod
    def decrypt_resource_info(resource_info):

        if not resource_info:
            raise ValueError
        if not isinstance(resource_info, str):
            raise TypeError

        [topic_name, callerids_str] = resource_info.split(' ', 1)

        checkInputData.str_data_valid(topic_name)

        if len(callerids_str) <= 2:
            return ['timestamp_obs_' + str(topic_name) + "_all"]

        callerids = [x.strip() for x in callerids_str[1:-1].split(',')]

        infos = []
        for callerid in callerids:
            checkInputData.str_data_valid(callerid)
            infos.append('timestamp_obs_' + str(topic_name) + "_" + str(callerid))

        return infos


picosat.SENTENCE_INTERPRETERS[TimestampObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timestamp'] = TimestampObserver


class TestTimestampObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_timestamp_observer1(self):
        ab_node = ab_pred("/node1")
        observation = "/topic"
        ab_subscribed_topics = all_ab_pred(['/topic1'])

        ab_node_tests = [
            (ValueError, ab_pred("")),
            (ValueError, ab_pred("/")),
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, ab_node) in ab_node_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_node) + "'",
                TimestampObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_timestamp_observer2(self):
        ab_node = ab_pred("/node1")
        observation = "/topic"
        ab_subscribed_topics = all_ab_pred(['/topic1'])

        observation_tests = [
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, observation) in observation_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(observation) + "'",
                TimestampObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_timestamp_observer3(self):
        ab_node = ab_pred("/node1")
        observation = "/topic"
        ab_subscribed_topics = all_ab_pred(['/topic1'])

        ab_subscribed_topics_tests = [
            (ValueError, ["/"]),
            (ValueError, all_ab_pred(["/"])),
            (TypeError, [1]),
            (TypeError, "/"),
            (TypeError, 1),
        ]

        for (error, ab_subscribed_topics) in ab_subscribed_topics_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_subscribed_topics) + "'",
                TimestampObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause(self):
        observer = TimestampObserver(ab_pred("name"), "/topic", [])
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")

        observer = TimestampObserver(ab_pred("name"), "/topic", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), "/topic", "Third literal in clause does not match!")

        observer = TimestampObserver(ab_pred("name"), "/topic", all_ab_pred(['/topic1', '/topic2']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic2"), "Third literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[3]), "/topic", "Fourth literal in clause does not match!")

    def test_generate_model_parameter1(self):
        # config = {'topics': ['/topic'], 'type': 'timestamp'}
        config = observer_configuration(type="timestamp", resource=['/topic'])
        topics_published_from_nodes = {'/topic': ['/node1', '/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topic'], '/node2': ['/topic']}
        nodes_subscribe_topics = {}

        vars, rules, nodes, real_nodes = TimestampObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                    topics_subscribed_from_nodes,
                                                                                    nodes_publish_topics,
                                                                                    nodes_subscribe_topics)

        vars_req = {'timestamp_obs_/topic_all': Variable('timestamp_obs_/topic_all', 1, None),
                    'timestamp_obs_/topic_/node1': Variable('timestamp_obs_/topic_/node1', 1, None),
                    'timestamp_obs_/topic_/node2': Variable('timestamp_obs_/topic_/node2', 1, None)
                    }

        self.assertEqual(len(vars), len(vars_req), "timestamp added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        subscribed_topics = []
        rules_req = [TimestampObserver(ab_pred('/node1'), 'timestamp_obs_/topic_/node1', subscribed_topics),
                     TimestampObserver(ab_pred('/node2'), 'timestamp_obs_/topic_/node2', subscribed_topics),
                     CalleridsObserver('timestamp_obs_/topic_all',
                                       ['timestamp_obs_/topic_/node1', 'timestamp_obs_/topic_/node2'])]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "timestamp added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "timestamp should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "timestamp should not add real nodes!")

    def test_generate_model_parameter2(self):
        # config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'timestamp'}
        config = observer_configuration(type="timestamp", resource=['/topic1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}
        vars, rules, nodes, real_nodes = TimestampObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                    topics_subscribed_from_nodes,
                                                                                    nodes_publish_topics,
                                                                                    nodes_subscribe_topics)

        vars_req = {'timestamp_obs_/topic1_all': Variable('timestamp_obs_/topic1_all', 1, None),
                    'timestamp_obs_/topic1_node1': Variable('timestamp_obs_/topic1_node1', 1, None),
                    # 'timestamp_obs_/topic2_all': Variable('timestamp_obs_/topic2_all', 1, None),
                    # 'timestamp_obs_/topic2_node2': Variable('timestamp_obs_/topic2_node2', 1, None),
                    # 'timestamp_obs_/topic3_all': Variable('timestamp_obs_/topic3_all', 1, None),
                    # 'timestamp_obs_/topic3_node3': Variable('timestamp_obs_/topic3_node3', 1, None),
                    }

        self.assertEqual(len(vars), len(vars_req), "timestamp added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [TimestampObserver(ab_pred('node1'), 'timestamp_obs_/topic1_node1', all_ab_pred([])),
                     CalleridsObserver('timestamp_obs_/topic1_all', ['timestamp_obs_/topic1_node1']),
                     # TimestampObserver(ab_pred('node2'), 'timestamp_obs_/topic2_node2', all_ab_pred(['/topic1'])),
                     # CalleridsObserver('timestamp_obs_/topic2_all', ['timestamp_obs_/topic2_node2']),
                     # TimestampObserver(ab_pred('node3'), 'timestamp_obs_/topic3_node3', all_ab_pred(['/topic2'])),
                     # CalleridsObserver('timestamp_obs_/topic3_all', ['timestamp_obs_/topic3_node3']),
                     ]

        rules_req_str = [str(x) for x in rules_req]

        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "timestamp added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "timestamp should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "timestamp should not add real nodes!")

    def test_generate_model_parameter_errors_1(self):
        # test different arguments for the config-parameter which all should raise exeptions
        # config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'timestamp'}
        config = observer_configuration(type="timestamp", resource=['/topic1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        config_tests = [(KeyError, {'topics_wrong_name': ['/topic'], 'type': 'timestamp'}),
                        # (KeyError, {'type': 'timestamp'}),
                        # (KeyError, {}),
                        # (TypeError, "not_a_dict"),
                        # (TypeError, 1),
                        # (ValueError, {'topics': [], 'type': 'timestamp'}),
                        # (ValueError, {'topics': [''], 'type': 'timestamp'}),
                        # (TypeError, {'topics': [1], 'type': 'timestamp'}),
                        # (TypeError, {'topics': "no_list", 'type': 'timestamp'}),
                        # (TypeError, {'topics': 1, 'type': 'timestamp'}),
                        # (ValueError, {'topics': ['/topic', '/topic2', '/topic3'], 'type': 'timestamp'})
                        ]
        config_tests = [(KeyError, observer_configuration(type="timestamp_wrong", resource=['/topic'])),
                        (ValueError, observer_configuration(type="timestamp")),
                        (KeyError, observer_configuration()),
                        (TypeError, observer_configuration),
                        (TypeError, 1),
                        (ValueError, observer_configuration(type="timestamp", resource=[''])),
                        (TypeError, observer_configuration(type="timestamp", resource=[1])),
                        (TypeError, observer_configuration(type="timestamp", resource='no_list')),
                        (TypeError, observer_configuration(type="timestamp", resource=1)),
                        (ValueError,
                         observer_configuration(type="timestamp", resource=['/topic1', '/topic2', '/topic3'])),
                        (ValueError, observer_configuration(type="timestamp", resource=['/topic_wrong']))
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                TimestampObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):
        # config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'timestamp'}
        config = observer_configuration(type="timestamp", resource=['/topic1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        topics_published_from_nodes_testes = \
            [
                # (ValueError, {'/topic':  ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': [], '/topic2': ['node2'], '/topic1': ['node1']}),
                # (ValueError, {'/topic3': ['node3'], '/topic':  ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': [], '/topic1': ['node1']}),
                # (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic':  ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': []}),
                (TypeError, {1: ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (KeyError, {}),
                (TypeError, "no_dict"),
                (TypeError, 1),
                (ValueError, {'/topic3': ['/', 'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['', 'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (TypeError, {'/topic3': [1, 'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3', '/'], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3', ''], '/topic2': ['node2'], '/topic1': ['node1']}),
                (TypeError, {'/topic3': ['node3', 1], '/topic2': ['node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['/', 'node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['', 'node2'], '/topic1': ['node1']}),
                (TypeError, {'/topic3': ['node3'], '/topic2': [1, 'node2'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2', '/'], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2', ''], '/topic1': ['node1']}),
                (TypeError, {'/topic3': ['node3'], '/topic2': ['node2', 1], '/topic1': ['node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['/', 'node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['', 'node1']}),
                (TypeError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': [1, 'node1']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', '/']}),
                (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', '']}),
                (TypeError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', 1]}),
            ]

        for (error, topics_published_from_nodes) in topics_published_from_nodes_testes:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(topics_published_from_nodes) + "'",

                TimestampObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_3(self):
        # config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'timestamp'}
        config = observer_configuration(type="timestamp", resource=['/topic1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        nodes_subscribe_topics_testes = \
            [(ValueError, {'node1': ['/'], 'node2': ['/topic1']}),
             (ValueError, {'node1': [''], 'node2': ['/topic1']}),
             (TypeError, {'node1': [1], 'node2': ['/topic1']}),
             (ValueError, {'node1': ['/topic1'], 'node2': ['/']}),
             (ValueError, {'node1': ['/topic1'], 'node2': ['']}),
             (TypeError, {'node1': ['/topic1'], 'node2': [1]}),
             (ValueError, {'node1': ['/wrong_topic_name'], 'node2': ['/topic2']}),
             ]

        for (error, nodes_subscribe_topics) in nodes_subscribe_topics_testes:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(nodes_subscribe_topics) + "'",

                TimestampObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1, node2]"),
                         ['timestamp_obs_/topic_name_node1', 'timestamp_obs_/topic_name_node2'],
                         "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1, node2, node3, node4, node5]"),
                         ['timestamp_obs_/topic_name_node1', 'timestamp_obs_/topic_name_node2',
                          'timestamp_obs_/topic_name_node3',
                          'timestamp_obs_/topic_name_node4', 'timestamp_obs_/topic_name_node5'],
                         "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1]"),
                         ['timestamp_obs_/topic_name_node1'], "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name {}"), ['timestamp_obs_/topic_name_all'],
                         "Topic name decryption not correct!")

        resource_info_tests = [
            (ValueError, "/ [node1, node2]"),
            (ValueError, "/ [node1, /]"),
            (ValueError, "/topic_name"),
            (ValueError, "/topic_name [/]"),
            (ValueError, "/topic_name [/node1, ]"),
            (ValueError, "/topic_name [/node1, /]"),
            (ValueError, "/"),
            (ValueError, ""),
            (TypeError, 1),
        ]

        for (error, resource_info) in resource_info_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(resource_info) + "'",
                TimestampObserver.decrypt_resource_info(resource_info)
            print "... DONE"
