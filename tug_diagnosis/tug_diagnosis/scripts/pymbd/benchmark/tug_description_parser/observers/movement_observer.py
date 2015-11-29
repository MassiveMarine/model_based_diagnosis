from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *

import re
regex_prog = re.compile('(\S+)\s+(\S+)')


class MovementObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics):
        super(MovementObserver, self).__init__()
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
        return "movement: %s, %s, %s, %s, %s)" % (self.ab_nodes_a, self.ab_nodes_b, self.ab_function, self.observation, self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos(self.ab_nodes_a + self.ab_nodes_b + self.ab_subscribed_topics) + " " + self.ab_function + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        checkInputData.dict_data_valid(config, check_entries=False)
        topics = config['topics']

        checkInputData.list_data_valid(topics, check_entries=False)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        nodes.append("movement")
        vars["movement"] = Variable("movement", Variable.BOOLEAN, None)
        vars[ab_pred("movement")] = Variable(ab_pred("movement"), Variable.BOOLEAN, None)

        for topic_pair in topics:
            checkInputData.list_data_valid(topic_pair, check_entries=True)
            topic_a = topic_pair[0]
            topic_b = topic_pair[1]

            observation = "movement_obs_" + topic_a + "_" + topic_b
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)

            nodes_a = topics_published_from_nodes.get(topic_a, [])
            nodes_b = topics_published_from_nodes.get(topic_b, [])
            checkInputData.list_data_valid(nodes_a, check_entries=True)
            checkInputData.list_data_valid(nodes_b, check_entries=True)

            subscribed_topics = []
            [subscribed_topics.extend(nodes_subscribe_topics.get(node, [])) for node in nodes_a + nodes_b]


            rules.append(MovementObserver(all_ab_pred(nodes_a), all_ab_pred(nodes_b), ab_pred("movement"), observation, all_ab_pred(subscribed_topics)))

            if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
                raise ValueError

        return vars, rules, nodes

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

        return ['movement_obs_' + topicA_name + "_" + topicB_name]


picosat.SENTENCE_INTERPRETERS[MovementObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['movement'] = MovementObserver


import unittest


class TestMovementObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_movement_observer1(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("movement")
        observation = "movement_obs_node1_node2"
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
                MovementObserver(ab_nodes, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

        for (error, ab_nodes) in ab_node_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_nodes) + "'",
                MovementObserver(ab_nodes_a, ab_nodes, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_movement_observer2(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("movement")
        observation = "movement_obs_node1_node2"
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
                MovementObserver(ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_movement_observer3(self):
        ab_nodes_a = [ab_pred("/node1")]
        ab_nodes_b = [ab_pred("/node2")]
        ab_function = ab_pred("movement")
        observation = "movement_obs_node1_node2"
        ab_subscribed_topics = []

        observation_tests = [
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, observation) in observation_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(observation) + "'",
                MovementObserver(ab_nodes_a, ab_nodes_b, ab_function, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause1(self):
        observer = MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("movement"), "movement_obs_node1_node2", all_ab_pred([]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("movement"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), "movement_obs_node1_node2", "A literal in clause does not match!")

    def test_clause2(self):
        observer = MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("movement"), "movement_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 5, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("movement"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), "movement_obs_node1_node2", "A literal in clause does not match!")

    def test_clause3(self):
        observer = MovementObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2")], ab_pred("movement"), "movement_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 6, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("movement"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[5]), "movement_obs_node1_node2", "A literal in clause does not match!")

    def test_clause4(self):
        observer = MovementObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2"), ab_pred("node4")], ab_pred("movement"), "movement_obs_node1_node2", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 7, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("node4"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[5]), ab_pred("movement"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[6]), "movement_obs_node1_node2", "A literal in clause does not match!")

    def test_clause5(self):
        observer = MovementObserver([ab_pred("node1"), ab_pred("node3")], [ab_pred("node2"), ab_pred("node4")], ab_pred("movement"), "movement_obs_node1_node2", all_ab_pred(['/topic1', '/topic2']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 8, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node3"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("node2"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), ab_pred("node4"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[4]), ab_pred("/topic1"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[5]), ab_pred("/topic2"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[6]), ab_pred("movement"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[7]), "movement_obs_node1_node2", "A literal in clause does not match!")

    def test_generate_model_parameter1(self):
        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        vars, rules, nodes = MovementObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)

        vars_req = {'movement': Variable("movement", Variable.BOOLEAN, None),
                    ab_pred("movement"): Variable(ab_pred("movement"), Variable.BOOLEAN, None),
                    'movement_obs_/topicA_/topicB': Variable('movement_obs_/topicA_/topicB', 1, None),
                    }

        self.assertEqual(len(vars), len(vars_req), "Movement added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        subscribed_topics = []
        rules_req = [(MovementObserver(all_ab_pred(['/node1']), all_ab_pred(['/node2']), ab_pred("movement"), 'movement_obs_/topicA_/topicB', all_ab_pred([])))]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "movement added wrong number of rules!")

        self.assertEqual(len(nodes), 1, "movement should not add nodes!")
        self.assertEqual(str(nodes[0]), 'movement', "'movement' not added to nodes!")

    def test_generate_model_parameter_errors_1(self):
        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        config_tests = [(KeyError, {'topics_wrong_name': [['/topicA', '/topicB']], 'type': 'movement'}),
                        (KeyError, {'type': 'movement'}),
                        (KeyError, {}),
                        (TypeError, "not_a_dict"),
                        (TypeError, 1),
                        (ValueError, {'topics': [[]], 'type': 'movement'}),
                        (ValueError, {'topics': [['']], 'type': 'movement'}),
                        (TypeError, {'topics': [[1]], 'type': 'movement'}),
                        (TypeError, {'topics': ["no_list_list"], 'type': 'movement'}),
                        (TypeError, {'topics': [1], 'type': 'movement'}),
                        (ValueError, {'topics': [['/wrong_topic_name', '/topic2', '/topic3']], 'type': 'movement'}),
                        (ValueError, {'topics': [], 'type': 'movement'}),
                        (TypeError, {'topics': [''], 'type': 'movement'}),
                        (TypeError, {'topics': [1], 'type': 'movement'}),
                        (TypeError, {'topics': "no_list", 'type': 'movement'}),
                        (TypeError, {'topics': 1, 'type': 'movement'}),
                        (TypeError, {'topics': ['/topic', '/topic2', '/topic3'], 'type': 'movement'})
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                MovementObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):

        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
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
                                             (ValueError, {'/topicB': [1, '/node2']}),
                                             (ValueError, {'/topicB': ['/node1', '/']}),
                                             (ValueError, {'/topicB': ['/node1', '']}),
                                             (ValueError, {'/topicB': ['/node1', 1]}),
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

                MovementObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_3(self):

        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
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

                MovementObserver.generate_model_parameter(config,
                                                          topics_published_from_nodes, topics_subscribed_from_nodes,
                                                          nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(MovementObserver.decrypt_resource_info("/topic_1 /topic_2"), ['movement_obs_/topic_1_/topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("topic_1 /topic_2"), ['movement_obs_topic_1_/topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("/topic_1 topic_2"), ['movement_obs_/topic_1_topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("topic_1 topic_2"), ['movement_obs_topic_1_topic_2'], "Topic name decryption not correct!")

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
                MovementObserver.decrypt_resource_info(resource_info)
            print "... DONE"
