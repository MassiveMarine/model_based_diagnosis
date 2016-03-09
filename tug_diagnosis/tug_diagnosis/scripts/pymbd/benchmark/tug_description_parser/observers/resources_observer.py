from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *
from tug_diagnosis_msgs.msg import observer_configuration
import unittest


class ResourcesObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, ab_node, observation, ab_subscribed_topics):
        super(ResourcesObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(observation)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)

        self.ab_node = ab_node
        self.observation = observation
        self.ab_subscribed_topics = ab_subscribed_topics

    def __repr__(self):
        return "resources: %s, %s, %s" % (self.ab_node, self.observation, self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos([self.ab_node] + self.ab_subscribed_topics) + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        config_type = str('resources')
        if not isinstance(config, observer_configuration):
            raise TypeError
        if not config.type == config_type:
            raise KeyError('given type in config is wrong!')

        nodes = config.resource
        checkInputData.list_data_valid(nodes, num_entries=1)

        # checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=True, entry_type=list, allow_empty=True)

        vars = {}
        rules = []

        node = nodes[0]
        observation = config_type + "_obs_" + node
        vars[observation] = Variable(observation, Variable.BOOLEAN, None)

        subscribed_topics = nodes_subscribe_topics.get(node, [])
        rules.append(ResourcesObserver(ab_pred(node), observation, all_ab_pred(subscribed_topics)))

        if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
            raise ValueError('subscribed topics are not not in topics published list!')

        return vars, rules, [], []

    @staticmethod
    def decrypt_resource_info(resource_info):
        checkInputData.str_data_valid(resource_info, forbidden_chars=[' '])

        return ['resources_obs_' + resource_info]


picosat.SENTENCE_INTERPRETERS[ResourcesObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['resources'] = ResourcesObserver


class TestResourcesObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_resource_observer1(self):
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
                ResourcesObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_resource_observer2(self):
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
                ResourcesObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_resource_observer3(self):
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
                ResourcesObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause(self):
        observer = ResourcesObserver(ab_pred("name"), "/obs_node", [])
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/obs_node", "Second literal in clause does not match!")

        observer = ResourcesObserver(ab_pred("name"), "/obs_node", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), "/obs_node", "Third literal in clause does not match!")

        observer = ResourcesObserver(ab_pred("name"), "/obs_node", all_ab_pred(['/topic1', '/topic2']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic2"), "Third literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[3]), "/obs_node", "Fourth literal in clause does not match!")

    def test_generate_model_parameter1(self):
        # config = {'nodes': ['/node1', '/node2', '/node3'], 'type': 'resources'}
        config = observer_configuration(type="resources", resource=['/node1'])
        topics_published_from_nodes = {'/topic': ['/node1', '/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topic'], '/node2': ['/topic']}
        nodes_subscribe_topics = {}
        vars, rules, nodes, real_nodes = ResourcesObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                    topics_subscribed_from_nodes,
                                                                                    nodes_publish_topics,
                                                                                    nodes_subscribe_topics)

        vars_req = {'resources_obs_/node1': Variable('resources_obs_/node1', 1, None),
                    # 'resources_obs_/node2': Variable('resources_obs_/node2', 1, None),
                    # 'resources_obs_/node3': Variable('resources_obs_/node3', 1, None)
                    }

        self.assertEqual(len(vars), len(vars_req), "resources added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        subscribed_topics = []
        rules_req = [ResourcesObserver(ab_pred('/node1'), 'resources_obs_/node1', subscribed_topics),
                     # ResourcesObserver(ab_pred('/node2'), 'resources_obs_/node2', subscribed_topics),
                     # ResourcesObserver(ab_pred('/node3'), 'resources_obs_/node3', subscribed_topics)
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "resources added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "resources should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "resources should not add real nodes!")

    def test_generate_model_parameter2(self):
        # config = {'nodes': ['node1', 'node2', 'node3'], 'type': 'resources'}
        config = observer_configuration(type="resources", resource=['node1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}
        vars, rules, nodes, real_nodes = ResourcesObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                    topics_subscribed_from_nodes,
                                                                                    nodes_publish_topics,
                                                                                    nodes_subscribe_topics)

        vars_req = {'resources_obs_node1': Variable('resources_obs_node1', 1, None),
                    # 'resources_obs_node2': Variable('resources_obs_node2', 1, None),
                    # 'resources_obs_node3': Variable('resources_obs_node3', 1, None)
                    }

        self.assertEqual(len(vars), len(vars_req), "resources added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [ResourcesObserver(ab_pred('node1'), 'resources_obs_node1', all_ab_pred([])),
                     # ResourcesObserver(ab_pred('node2'), 'resources_obs_node2', all_ab_pred(['/topic1'])),
                     # ResourcesObserver(ab_pred('node3'), 'resources_obs_node3', all_ab_pred(['/topic2']))
                     ]

        rules_req_str = [str(x) for x in rules_req]

        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "resources added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "resources should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "resources should not add real nodes!")

    def test_generate_model_parameter_errors_1(self):
        # config = {'nodes': ['node1', 'node2', 'node3'], 'type': 'resources'}
        config = observer_configuration(type="resources", resource=['node1'])
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        # config_tests = [(KeyError, {'nodes_wrong_name': ['node1'], 'type': 'resources'}),
        #                 (KeyError, {'type': 'resources'}),
        #                 (KeyError, {}),
        #                 (TypeError, "not_a_dict"),
        #                 (TypeError, 1),
        #                 (ValueError, {'nodes': [], 'type': 'resources'}),
        #                 (ValueError, {'nodes': [''], 'type': 'resources'}),
        #                 (TypeError, {'nodes': [1], 'type': 'resources'}),
        #                 (TypeError, {'nodes': "no_list", 'type': 'resources'}),
        #                 (TypeError, {'nodes': 1, 'type': 'resources'})
        #                 ]

        config_tests = [(KeyError, observer_configuration(type="resources_wrong", resource=['/node'])),
                        (ValueError, observer_configuration(type="resources")),
                        (KeyError, observer_configuration()),
                        (TypeError, observer_configuration),
                        (TypeError, 1),
                        (ValueError, observer_configuration(type="resources", resource=[''])),
                        (TypeError, observer_configuration(type="resources", resource=[1])),
                        (TypeError, observer_configuration(type="resources", resource='no_list')),
                        (TypeError, observer_configuration(type="resources", resource=1)),
                        (ValueError, observer_configuration(type="resources", resource=['/node1', '/node2', '/node3'])),
                        # (ValueError, observer_configuration(type="resources", resource=['/node_wrong']))
                        ]
        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                ResourcesObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):
        # config = {'nodes': ['node1', 'node2', 'node3'], 'type': 'resources'}
        config = observer_configuration(type="resources", resource=['node1'])
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

                ResourcesObserver.generate_model_parameter(config,
                                                           topics_published_from_nodes, topics_subscribed_from_nodes,
                                                           nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resources_info(self):
        self.assertEqual(ResourcesObserver.decrypt_resource_info("/node1"), ['resources_obs_/node1'],
                         "Topic name decryption not correct!")

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
                ResourcesObserver.decrypt_resource_info(resource_info)
            print "... DONE"
