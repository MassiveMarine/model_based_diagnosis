from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class ActivatedObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """
    def __init__(self, ab_node, observation):
        super(ActivatedObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(observation)

        self.ab_node = ab_node
        self.observation = observation

    def __repr__(self):
        return "activated: %s, %s" % (self.ab_node, self.observation)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        checkInputData.dict_data_valid(config, False)
        nodes = config['nodes']

        checkInputData.list_data_valid(nodes)

        vars = {}
        rules = []

        for node in nodes:
            observation = "activated_obs_" + node
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)
            rules.append(ActivatedObserver(ab_pred(str(node)), observation))

        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(resource_info):
        checkInputData.str_data_valid(resource_info, forbidden_chars=[' '])
        return ['activated_obs_' + resource_info]


picosat.SENTENCE_INTERPRETERS[ActivatedObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['activated'] = ActivatedObserver


import unittest


class TestActivatedObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_activated_observer1(self):
        ab_node = ab_pred("/node1")
        observation = "/node1"

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
                ActivatedObserver(ab_node, observation)
            print "... DONE"

    def test_activated_observer2(self):
        ab_node = ab_pred("/node1")
        observation = "/topic"

        observation_tests = [
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, observation) in observation_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(observation) + "'",
                ActivatedObserver(ab_node, observation)
            print "... DONE"

    def test_clause(self):
        observer = ActivatedObserver(ab_pred("name"), "/name")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/name", "Second literal in clause does not match!")

    def test_generate_model_parameter1(self):
        config = {'nodes': ['/node1', '/node2', '/node3'], 'type': 'activated'}
        topics_published_from_nodes = {'/topic': ['/node1', '/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topic'], '/node2': ['/topic']}
        nodes_subscribe_topics = {}
        vars, rules, nodes = ActivatedObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)

        vars_req = {'activated_obs_/node1': Variable('activated_obs_/node1', 1, None),
                    'activated_obs_/node2': Variable('activated_obs_/node2', 1, None),
                    'activated_obs_/node3': Variable('activated_obs_/node3', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "Activated added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [ActivatedObserver(ab_pred('/node1'), 'activated_obs_/node1'),
                     ActivatedObserver(ab_pred('/node2'), 'activated_obs_/node2'),
                     ActivatedObserver(ab_pred('/node3'), 'activated_obs_/node3')]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Activated added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Activated should not add nodes!")

    def test_generate_model_parameter_errors_1(self):
        config = {'nodes': ['node1', 'node2', 'node3'], 'type': 'activated'}
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        config_tests = [(KeyError, {'nodes_wrong_name': ['node1'], 'type': 'activated'}),
                        (KeyError, {'type': 'activated'}),
                        (KeyError, {}),
                        (TypeError, "not_a_dict"),
                        (TypeError, 1),
                        (ValueError, {'nodes': [], 'type': 'activated'}),
                        (ValueError, {'nodes': [''], 'type': 'activated'}),
                        (TypeError, {'nodes': [1], 'type': 'activated'}),
                        (TypeError, {'nodes': "no_list", 'type': 'activated'}),
                        (TypeError, {'nodes': 1, 'type': 'activated'})
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                ActivatedObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(ActivatedObserver.decrypt_resource_info("/node1"), ['activated_obs_/node1'], "Topic name decryption not correct!")

        resource_info_tests = [
            (ValueError, "/"),
            (ValueError, ""),
            (ValueError, "/node_name "),
            (ValueError, "/node_name []"),
            (ValueError, "/node_name some_wrong_text"),
            (TypeError, 1),
            ]

        for (error, resource_info) in resource_info_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(resource_info) + "'",
                ActivatedObserver.decrypt_resource_info(resource_info)
            print "... DONE"

