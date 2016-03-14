from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *
from tug_diagnosis_msgs.msg import observer_configuration

import re

import unittest

regex_prog = re.compile('(\S+)\s+(\S+)')


class VelocityObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics):
        super(VelocityObserver, self).__init__()
        checkInputData.list_data_valid(ab_nodes_a)
        checkInputData.list_data_valid(ab_nodes_b)
        checkInputData.str_data_valid(ab_function)
        checkInputData.str_data_valid(observation)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)

        self.ab_nodes_a = ab_nodes_a
        self.ab_nodes_b = ab_nodes_b
        self.ab_function = ab_function
        self.observation = observation
        self.ab_subscribed_topics = ab_subscribed_topics

    def __repr__(self):
        return "velocity: %s, %s, %s, %s, %s)" % (self.ab_nodes_a, self.ab_nodes_b, self.ab_function, self.observation,
                                                  self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos(self.ab_nodes_a + self.ab_nodes_b + self.ab_subscribed_topics) + " " +
                       self.ab_function + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        config_type = str('velocity')
        if not isinstance(config, observer_configuration):
            raise TypeError
        if not config.type == config_type:
            raise KeyError('given type in config is wrong!')

        topics = config.resource
        checkInputData.list_data_valid(topics, num_entries=2)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=True, entry_type=list,
                                       allow_empty=False)
        checkInputData.list_data_valid(topics_published_from_nodes.keys(), check_entries=True)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=True, entry_type=list, allow_empty=True)

        vars = {}
        rules = []
        real_nodes = []

        real_nodes.append(config_type)
        vars[config_type] = Variable(config_type, Variable.BOOLEAN, None)
        vars[ab_pred(config_type)] = Variable(ab_pred(config_type), Variable.BOOLEAN, None)

        topic_pair = topics
        checkInputData.list_data_valid(topic_pair, check_entries=True, allow_empty=False)
        topic_a = topic_pair[0]
        topic_b = topic_pair[1]

        if not set(topic_pair).issubset(topics_published_from_nodes.keys()):
            raise ValueError('topics are not not in topics published list!')

        observation = config_type + "_obs_" + topic_a + "_" + topic_b
        vars[observation] = Variable(observation, Variable.BOOLEAN, None)

        nodes_a = topics_published_from_nodes.get(topic_a, [])
        nodes_b = topics_published_from_nodes.get(topic_b, [])
        checkInputData.list_data_valid(nodes_a, check_entries=True, allow_empty=False)
        checkInputData.list_data_valid(nodes_b, check_entries=True, allow_empty=False)

        subscribed_topics = []
        [subscribed_topics.extend(nodes_subscribe_topics.get(node, [])) for node in nodes_a + nodes_b]

        rules.append(VelocityObserver(all_ab_pred(nodes_a), all_ab_pred(nodes_b), ab_pred(config_type), observation,
                                      all_ab_pred(subscribed_topics)))

        if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
            raise ValueError('subscribed topics are not not in topics published list!')

        return vars, rules, [], real_nodes

    @staticmethod
    def decrypt_resource_info(resource_info):
        if not resource_info:
            raise ValueError
        if not isinstance(resource_info, str):
            raise TypeError

        entries = re.findall(regex_prog, resource_info)
        if not len(entries) or len(entries) > 1:
            raise ValueError

        [topicA_name, topicB_name] = list(re.findall(regex_prog, resource_info)[0])

        checkInputData.str_data_valid(topicA_name)
        checkInputData.str_data_valid(topicB_name)

        return ['velocity_obs_' + topicA_name + "_" + topicB_name]


picosat.SENTENCE_INTERPRETERS[VelocityObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['velocity'] = VelocityObserver


class TestVelocityObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_velocity_observer1(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("velocity")
        observation = "velocity_obs_node1_node2"
        ab_subscribed_topics = []

        ab_node_tests = [(ValueError, [ab_pred("")]),
                         (ValueError, [ab_pred("/")]),
                         (ValueError, [""]),
                         (ValueError, ["/"]),
                         (ValueError, []),
                         (TypeError, [1]),
                         (TypeError, ""),
                         (TypeError, "/"),
                         (TypeError, "node1"),
                         (TypeError, 1)]

        for (error, ab_nodes) in ab_node_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_nodes) + "'",
                VelocityObserver(ab_nodes, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

        for (error, ab_nodes) in ab_node_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_nodes) + "'",
                VelocityObserver(ab_nodes_a, ab_nodes, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_velocity_observer2(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("velocity")
        observation = "velocity_obs_node1_node2"
        ab_subscribed_topics = []

        ab_function_tests = [
            (ValueError, ab_pred("")),
            (ValueError, ab_pred("/")),
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, ab_function) in ab_function_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_function) + "'",
                VelocityObserver(ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_velocity_observer3(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("velocity")
        observation = "velocity_obs_node1_node2"
        ab_subscribed_topics = []

        observation_tests = [
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, observation) in observation_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(observation) + "'",
                VelocityObserver(ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause1(self):
        observer = VelocityObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("velocity"),
                                    "velocity_obs_node1_node2", all_ab_pred([]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("velocity"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), "velocity_obs_node1_node2", "A literal in clause does not match!")

    def test_clause2(self):
        observer = VelocityObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("velocity"),
                                    "velocity_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 5, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("velocity"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), "velocity_obs_node1_node2", "A literal in clause does not match!")

    def test_clause3(self):
        observer = VelocityObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2")], ab_pred("velocity"),
                                    "velocity_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 6, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("velocity"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[5]), "velocity_obs_node1_node2", "A literal in clause does not match!")

    def test_clause4(self):
        observer = VelocityObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2"), ab_pred("node4")],
                                    ab_pred("velocity"), "velocity_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 7, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("node4"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[5]), ab_pred("velocity"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[6]), "velocity_obs_node1_node2", "A literal in clause does not match!")

    def test_clause5(self):
        observer = VelocityObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2"), ab_pred("node4")],
                                    ab_pred("velocity"), "velocity_obs_node1_node2",
                                    all_ab_pred(['/topic1', '/topic2']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 8, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("node4"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[5]), ab_pred("/topic2"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[6]), ab_pred("velocity"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[7]), "velocity_obs_node1_node2", "A literal in clause does not match!")

    def test_generate_model_parameter1(self):
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'velocity'}
        config = observer_configuration(type="velocity", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        vars, rules, nodes, real_nodes = VelocityObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                   topics_subscribed_from_nodes,
                                                                                   nodes_publish_topics,
                                                                                   nodes_subscribe_topics)

        vars_req = {'velocity': Variable("velocity", Variable.BOOLEAN, None),
                    ab_pred("velocity"): Variable(ab_pred("velocity"), Variable.BOOLEAN, None),
                    'velocity_obs_/topicA_/topicB': Variable('velocity_obs_/topicA_/topicB', 1, None),
                    }

        self.assertEqual(len(vars), len(vars_req), "Velocity added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        subscribed_topics = []
        rules_req = [(VelocityObserver(all_ab_pred(['/node1']), all_ab_pred(['/node2']), ab_pred("velocity"),
                                       'velocity_obs_/topicA_/topicB', all_ab_pred([])))]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "velocity added wrong number of rules!")

        self.assertEqual(len(nodes), 0, "Velocity should not add nodes!")
        self.assertEqual(len(real_nodes), 1, "Velocity should add one real node!")
        self.assertEqual(str(real_nodes[0]), 'velocity', "'Velocity' not added to real nodes!")

    def test_generate_model_parameter_errors_1(self):
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'velocity'}
        config = observer_configuration(type="velocity", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        config_tests = [(KeyError, {'topics_wrong_name': [['/topicA', '/topicB']], 'type': 'velocity'}),
                        (KeyError, {'type': 'velocity'}),
                        (KeyError, {}),
                        (TypeError, "not_a_dict"),
                        (TypeError, 1),
                        (ValueError, {'topics': [[]], 'type': 'velocity'}),
                        (ValueError, {'topics': [['']], 'type': 'velocity'}),
                        (TypeError, {'topics': [[1]], 'type': 'velocity'}),
                        (TypeError, {'topics': ["no_list_list"], 'type': 'velocity'}),
                        (TypeError, {'topics': [1], 'type': 'velocity'}),
                        (ValueError, {'topics': [['/wrong_topic_name', '/topic2', '/topic3']], 'type': 'velocity'}),
                        (ValueError, {'topics': [], 'type': 'velocity'}),
                        (TypeError, {'topics': [''], 'type': 'velocity'}),
                        (TypeError, {'topics': [1], 'type': 'velocity'}),
                        (TypeError, {'topics': "no_list", 'type': 'velocity'}),
                        (TypeError, {'topics': 1, 'type': 'velocity'}),
                        (TypeError, {'topics': ['/topic', '/topic2', '/topic3'], 'type': 'velocity'})
                        ]
        config_tests = [(KeyError, observer_configuration(type="velocity_wrong", resource=['/topicA', '/topicB'])),
                        (ValueError, observer_configuration(type="velocity")),
                        (KeyError, observer_configuration()),
                        (TypeError, observer_configuration),
                        (TypeError, 1),
                        (ValueError, observer_configuration(type="velocity", resource=[''])),
                        (ValueError, observer_configuration(type="velocity", resource=[1])),
                        (TypeError, observer_configuration(type="velocity", resource='no_list')),
                        (TypeError, observer_configuration(type="velocity", resource=1)),
                        (ValueError,
                         observer_configuration(type="velocity", resource=['/topicA', '/topicB', '/topicC'])),
                        (ValueError, observer_configuration(type="velocity", resource=['/topicA'])),
                        (ValueError, observer_configuration(type="velocity", resource=['/topic_wrong']))
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                VelocityObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):

        # config = {'topics': [['/topicA', '/topicB']], 'type': 'velocity'}
        config = observer_configuration(type="velocity", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        topics_published_from_nodes_tests = [(ValueError, {'/topic_wrong_name': ['/node1', '/node2']}),
                                             (ValueError, {'/topicB': []}),
                                             (KeyError, {}),
                                             (TypeError, "no_dict"),
                                             (TypeError, 1),
                                             (ValueError, {'/topicB': ['/', '/node2']}),
                                             (ValueError, {'/topicB': ['', '/node2']}),
                                             (TypeError, {'/topicB': [1, '/node2']}),
                                             (ValueError, {'/topicB': ['/node1', '/']}),
                                             (ValueError, {'/topicB': ['/node1', '']}),
                                             (TypeError, {'/topicB': ['/node1', 1]}),
                                             (ValueError, {'/topicB': ['/', '/node2'], '/topicA': ['/nodeA1']}),
                                             (ValueError, {'/topicB': ['', '/node2'], '/topicA': ['/nodeA1']}),
                                             (TypeError, {'/topicB': [1, '/node2'], '/topicA': ['/nodeA1']}),
                                             (ValueError, {'/topicB': ['/node1', '/'], '/topicA': ['/nodeA1']}),
                                             (ValueError, {'/topicB': ['/node1', ''], '/topicA': ['/nodeA1']}),
                                             (TypeError, {'/topicB': ['/node1', 1], '/topicA': ['/nodeA1']}),
                                             (ValueError, {'/topicB': ['/node2'], '/topicA': ['/', '/node1']}),
                                             (ValueError, {'/topicB': ['/node2'], '/topicA': ['', '/node1']}),
                                             (TypeError, {'/topicB': ['/node2'], '/topicA': [1, '/node1']}),
                                             (ValueError, {'/topicB': ['/node2'], '/topicA': ['/node1', '/']}),
                                             (ValueError, {'/topicB': ['/node2'], '/topicA': ['/node1', '']}),
                                             (TypeError, {'/topicB': ['/node2'], '/topicA': ['/node1', 1]})]

        for (error, topics_published_from_nodes) in topics_published_from_nodes_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(topics_published_from_nodes) + "'",

                VelocityObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_3(self):
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'velocity'}
        config = observer_configuration(type="velocity", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        nodes_subscribe_topics_testes = \
            [(ValueError, {'/node1': ['/'], '/node2': ['/topicB']}),
             (ValueError, {'/node1': [''], '/node2': ['/topicB']}),
             (TypeError, {'/node1': [1], '/node2': ['/topicB']}),
             (ValueError, {'/node1': ['/topicA'], '/node2': ['/']}),
             (ValueError, {'/node1': ['/topicA'], '/node2': ['']}),
             (TypeError, {'/node1': ['/topicA'], '/node2': [1]}),
             (ValueError, {'/node1': ['/wrong_topic_name'], '/node2': ['/topicB']}),
             ]

        for (error, nodes_subscribe_topics) in nodes_subscribe_topics_testes:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(nodes_subscribe_topics) + "'",

                VelocityObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(VelocityObserver.decrypt_resource_info("/topic_1 /topic_2"),
                         ['velocity_obs_/topic_1_/topic_2'], "Topic name decryption not correct!")
        self.assertEqual(VelocityObserver.decrypt_resource_info("topic_1 /topic_2"), ['velocity_obs_topic_1_/topic_2'],
                         "Topic name decryption not correct!")
        self.assertEqual(VelocityObserver.decrypt_resource_info("/topic_1 topic_2"), ['velocity_obs_/topic_1_topic_2'],
                         "Topic name decryption not correct!")
        self.assertEqual(VelocityObserver.decrypt_resource_info("topic_1 topic_2"), ['velocity_obs_topic_1_topic_2'],
                         "Topic name decryption not correct!")

        resource_info_tests = [
            (ValueError, "/ /topic_2"),
            (ValueError, "/topic_1_ /"),
            (ValueError, "/ /"),
            (ValueError, " /topic_name"),
            (ValueError, "/topic_name "),
            (ValueError, "/"),
            (ValueError, ""),
            (TypeError, 1),
            (ValueError, "/topic_1 [] /topic_2 []")
        ]

        for (error, resource_info) in resource_info_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(resource_info) + "'",
                VelocityObserver.decrypt_resource_info(resource_info)
            print "... DONE"
