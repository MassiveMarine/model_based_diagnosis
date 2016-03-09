from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *
from tug_diagnosis_msgs.msg import observer_configuration

import re

import unittest

regex_prog = re.compile('(\S*)\s?(\[\S*\s?\S*\s?\])\s?(\S*)\s?(\[\S*\s?\S*\s?\])')


class TimingObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, ab_nodes, topic, ab_subscribed_topics):
        super(TimingObserver, self).__init__()
        checkInputData.list_data_valid(ab_nodes, check_entries=True)
        checkInputData.str_data_valid(topic)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)

        self.ab_nodes = ab_nodes
        self.topic = topic
        self.ab_subscribed_topics = ab_subscribed_topics

    def __repr__(self):
        return "timing: %s, %s, %s" % (self.ab_nodes, self.topic, self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos(self.ab_nodes + self.ab_subscribed_topics) + " " + self.topic)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        config_type = str('timing')
        if not isinstance(config, observer_configuration):
            raise TypeError
        if not config.type == config_type:
            raise KeyError('given type in config is wrong!')

        topics = config.resource
        checkInputData.list_data_valid(topics, num_entries=2)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=True, entry_type=list,
                                       allow_empty=False)
        checkInputData.list_data_valid(topics_published_from_nodes.keys(), check_entries=True)
        checkInputData.dict_data_valid(topics_subscribed_from_nodes, check_entries=True, entry_type=list,
                                       allow_empty=True)
        checkInputData.list_data_valid(topics_subscribed_from_nodes.keys(), check_entries=True, allow_empty=True)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=True, entry_type=list, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        topic_pair = topics[:]
        checkInputData.list_data_valid(topic_pair, check_entries=True, allow_empty=False)
        topicA = topic_pair[0]
        topicB = topic_pair[1]

        if not set(topic_pair).issubset(topics_published_from_nodes.keys()):
            raise ValueError('topics are not not in topics published list!')

        nodes_list = []
        nodes_a = topics_published_from_nodes.get(topicA, [])
        nodes_b = topics_published_from_nodes.get(topicB, [])
        checkInputData.list_data_valid(nodes_a, check_entries=True, allow_empty=False)
        checkInputData.list_data_valid(nodes_b, check_entries=True, allow_empty=False)

        topics_to_node_a = topics_subscribed_from_nodes.get(topicA, [])
        topics_to_node_b = topics_subscribed_from_nodes.get(topicB, [])
        checkInputData.list_data_valid(topics_to_node_a, check_entries=True, allow_empty=True)
        checkInputData.list_data_valid(topics_to_node_b, check_entries=True, allow_empty=True)

        for node in nodes_b:
            if node in topics_to_node_a:
                nodes_list += nodes_b
                break

        for node in nodes_a:
            if node in topics_to_node_b:
                nodes_list += nodes_a

        if not nodes_list:
            nodes_list = nodes_a + nodes_b

        for calleridA in nodes_a:
            for calleridB in nodes_b:

                observation = config_type + "_obs_" + topicA + "_" + calleridA + "_" + topicB + "_" + calleridB
                vars[observation] = Variable(observation, Variable.BOOLEAN, None)

                subscribed_topics = nodes_subscribe_topics.get(calleridB, [])
                checkInputData.list_data_valid(subscribed_topics, allow_empty=True)

                rules.append(TimingObserver(all_ab_pred(nodes_list), observation, all_ab_pred(subscribed_topics)))

                if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
                    raise ValueError('subscribed topics are not not in topics published list!')

        new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter_2(config_type, topicA,
                                                                                      topics_published_from_nodes[
                                                                                          topicA], topicB,
                                                                                      topics_published_from_nodes[
                                                                                          topicB])
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

        [topicA_name, calleridsA_str, topicB_name, calleridsB_str] = list(re.findall(regex_prog, resource_info)[0])

        checkInputData.str_data_valid(topicA_name)
        checkInputData.str_data_valid(topicB_name)

        infos = []
        if len(calleridsA_str) <= 2:
            calleridsA = ["all"]
        else:
            calleridsA = [x.strip() for x in calleridsA_str[1:-1].split(',')]

        if len(calleridsB_str) <= 2:
            calleridsB = ["all"]
        else:
            calleridsB = [x.strip() for x in calleridsB_str[1:-1].split(',')]

        for calleridA in calleridsA:
            checkInputData.str_data_valid(calleridA)
            for calleridB in calleridsB:
                checkInputData.str_data_valid(calleridB)
                infos.append(
                    'timing_obs_' + str(topicA_name) + "_" + str(calleridA) + "_" + str(topicB_name) + "_" + str(
                        calleridB))

        return infos


picosat.SENTENCE_INTERPRETERS[TimingObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timing'] = TimingObserver


class TestTimingObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_timing_observer1(self):
        ab_nodes = [ab_pred("/node1")]
        observation = "timing_obs_node1_node2"
        ab_subscribed_topics = []

        ab_nodes_tests = [(ValueError, [ab_pred("")]),
                          (ValueError, [ab_pred("/")]),
                          (ValueError, [""]),
                          (ValueError, ["/"]),
                          (ValueError, []),
                          (TypeError, [1]),
                          (TypeError, ""),
                          (TypeError, "/"),
                          (TypeError, "node1"),
                          (TypeError, 1)]

        for (error, ab_nodes) in ab_nodes_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_nodes) + "'",
                TimingObserver(ab_nodes, observation, ab_subscribed_topics)
            print "... DONE"

    def test_timing_observe2(self):
        ab_nodes = [ab_pred("/node1")]
        observation = "timing_obs_node1_node2"
        ab_subscribed_topics = []

        observation_tests = [
            (ValueError, ""),
            (ValueError, "/"),
            (TypeError, 1),
        ]

        for (error, observation) in observation_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(observation) + "'",
                TimingObserver(ab_nodes, observation, ab_subscribed_topics)
            print "... DONE"

    def test_timing_observe3(self):
        ab_nodes = [ab_pred("/node1")]
        observation = "timing_obs_node1_node2"
        ab_subscribed_topics = []

        ab_subscribed_topics_tests = [(ValueError, [ab_pred("")]),
                                      (ValueError, [ab_pred("/")]),
                                      (ValueError, [""]),
                                      (ValueError, ["/"]),
                                      (TypeError, [1]),
                                      (TypeError, ""),
                                      (TypeError, "/"),
                                      (TypeError, "node1"),
                                      (TypeError, 1)]

        for (error, ab_subscribed_topics) in ab_subscribed_topics_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(ab_subscribed_topics) + "'",
                TimingObserver(ab_nodes, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause1(self):
        observer = TimingObserver([ab_pred("name1")], "/topic", all_ab_pred([]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "A literal in clause does not match!")

    def test_clause2(self):
        observer = TimingObserver([ab_pred("name1")], "/topic", all_ab_pred(["/topic_pre"]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic_pre"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), "/topic", "A literal in clause does not match!")

    def test_clause3(self):
        observer = TimingObserver([ab_pred("name1"), ab_pred("name2")], "/topic", all_ab_pred([]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("name2"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), "/topic", "A literal in clause does not match!")

    def test_clause4(self):
        observer = TimingObserver([ab_pred("name1"), ab_pred("name2")], "/topic", all_ab_pred(["/topic_pre"]))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("name2"), "A literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic_pre"), "A literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[3]), "/topic", "A literal in clause does not match!")

    def test_generate_model_parameter1(self):

        # +----------+ /topicA  +---------+ /topicB
        # | starter1 |--------->| timing1 |------->
        # +----------+          +---------+
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['node1'], '/topicB': ['node2']}
        topics_subscribed_from_nodes = {'/topicA': ['node2']}
        nodes_publish_topics = {'node1': ['/topicA'], 'node2': ['/topicB']}
        nodes_subscribe_topics = {'node2': ['/topicA']}
        vars, rules, nodes, real_nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                 topics_subscribed_from_nodes,
                                                                                 nodes_publish_topics,
                                                                                 nodes_subscribe_topics)

        vars_req = {'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_node2': Variable('timing_obs_/topicA_all_/topicB_node2', 1, None),
                    'timing_obs_/topicA_node1_/topicB_all': Variable('timing_obs_/topicA_node1_/topicB_all', 1, None),
                    'timing_obs_/topicA_node1_/topicB_node2': Variable('timing_obs_/topicA_node1_/topicB_node2', 1,
                                                                       None)}

        subscribed_topics = nodes_subscribe_topics.get('node2', [])
        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [TimingObserver(all_ab_pred(['node2']), 'timing_obs_/topicA_node1_/topicB_node2',
                                    all_ab_pred(subscribed_topics)),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_node2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_node1_/topicB_all']),
                     CalleridsObserver('timing_obs_/topicA_node1_/topicB_all',
                                       ['timing_obs_/topicA_node1_/topicB_node2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_node2',
                                       ['timing_obs_/topicA_node1_/topicB_node2']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "Timing should not add real nodes!")

    def test_generate_model_parameter2(self):
        # +----------+ /topicA
        # | starter1 |--------->
        # +----------+
        #
        # +----------+ /topicB
        # | starter2 |--------->
        # +----------+
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['node1'], '/topicB': ['node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'node1': ['/topicA'], 'node2': ['/topicB']}
        nodes_subscribe_topics = {}
        vars, rules, nodes, real_nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                 topics_subscribed_from_nodes,
                                                                                 nodes_publish_topics,
                                                                                 nodes_subscribe_topics)

        vars_req = {'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_node2': Variable('timing_obs_/topicA_all_/topicB_node2', 1, None),
                    'timing_obs_/topicA_node1_/topicB_all': Variable('timing_obs_/topicA_node1_/topicB_all', 1, None),
                    'timing_obs_/topicA_node1_/topicB_node2': Variable('timing_obs_/topicA_node1_/topicB_node2', 1,
                                                                       None)}

        subscribed_topics = nodes_subscribe_topics.get('node2', [])
        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [TimingObserver(all_ab_pred(['node1', 'node2']), 'timing_obs_/topicA_node1_/topicB_node2',
                                    all_ab_pred(subscribed_topics)),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_node2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_node1_/topicB_all']),
                     CalleridsObserver('timing_obs_/topicA_node1_/topicB_all',
                                       ['timing_obs_/topicA_node1_/topicB_node2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_node2',
                                       ['timing_obs_/topicA_node1_/topicB_node2']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "Timing should not add real nodes!")

    def test_generate_model_parameter3(self):
        # +----------+ /topicA         +---------+ /topicB
        # | starter1 |----------+  +-->| timing1 |----------+
        # +----------+          |  |   +---------+          |
        #                       *--*                        +--->
        # +----------+ /topicA  |  |   +---------+ /topicB  |
        # | starter2 |---------->  +-->| timing2 |----------+
        # +----------+                 +---------+
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['node1', 'node3'], '/topicB': ['node2', 'node4']}
        topics_subscribed_from_nodes = {'/topicA': ['node2', 'node4']}
        nodes_publish_topics = {'node1': ['/topicA'], 'node2': ['/topicB'], 'node3': ['/topicA'], 'node4': ['/topicB']}
        nodes_subscribe_topics = {'node2': ['/topicA'], 'node4': ['/topicA']}
        vars, rules, nodes, real_nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes,
                                                                                 topics_subscribed_from_nodes,
                                                                                 nodes_publish_topics,
                                                                                 nodes_subscribe_topics)

        vars_req = {}
        vars_req = {'timing_obs_/topicA_node1_/topicB_all': Variable('timing_obs_/topicA_node1_/topicB_all', 1, None),
                    'timing_obs_/topicA_node3_/topicB_node4': Variable('timing_obs_/topicA_node3_/topicB_node4', 1,
                                                                       None),
                    'timing_obs_/topicA_node3_/topicB_all': Variable('timing_obs_/topicA_node3_/topicB_all', 1, None),
                    'timing_obs_/topicA_node3_/topicB_node2': Variable('timing_obs_/topicA_node3_/topicB_node2', 1,
                                                                       None),
                    'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_node4': Variable('timing_obs_/topicA_all_/topicB_node4', 1, None),
                    'timing_obs_/topicA_all_/topicB_node2': Variable('timing_obs_/topicA_all_/topicB_node2', 1, None),
                    'timing_obs_/topicA_node1_/topicB_node2': Variable('timing_obs_/topicA_node1_/topicB_node2', 1,
                                                                       None),
                    'timing_obs_/topicA_node1_/topicB_node4': Variable('timing_obs_/topicA_node1_/topicB_node4', 1,
                                                                       None),
                    }

        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [TimingObserver(all_ab_pred(['node2', 'node4']), 'timing_obs_/topicA_node1_/topicB_node2',
                                    all_ab_pred(nodes_subscribe_topics.get('node2', []))),
                     TimingObserver(all_ab_pred(['node2', 'node4']), 'timing_obs_/topicA_node1_/topicB_node4',
                                    all_ab_pred(nodes_subscribe_topics.get('node4', []))),
                     TimingObserver(all_ab_pred(['node2', 'node4']), 'timing_obs_/topicA_node3_/topicB_node2',
                                    all_ab_pred(nodes_subscribe_topics.get('node2', []))),
                     TimingObserver(all_ab_pred(['node2', 'node4']), 'timing_obs_/topicA_node3_/topicB_node4',
                                    all_ab_pred(nodes_subscribe_topics.get('node4', []))),

                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_node2',
                                                                              'timing_obs_/topicA_all_/topicB_node4']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_node1_/topicB_all',
                                                                              'timing_obs_/topicA_node3_/topicB_all']),

                     CalleridsObserver('timing_obs_/topicA_node1_/topicB_all',
                                       ['timing_obs_/topicA_node1_/topicB_node2',
                                        'timing_obs_/topicA_node1_/topicB_node4']),
                     CalleridsObserver('timing_obs_/topicA_node3_/topicB_all',
                                       ['timing_obs_/topicA_node3_/topicB_node2',
                                        'timing_obs_/topicA_node3_/topicB_node4']),

                     CalleridsObserver('timing_obs_/topicA_all_/topicB_node2',
                                       ['timing_obs_/topicA_node1_/topicB_node2',
                                        'timing_obs_/topicA_node3_/topicB_node2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_node4',
                                       ['timing_obs_/topicA_node1_/topicB_node4',
                                        'timing_obs_/topicA_node3_/topicB_node4']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")
        self.assertEqual(len(real_nodes), 0, "Timing should not add real nodes!")

    def test_generate_model_parameter_errors_1(self):
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['/node1'], '/topicB': ['/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topicA'], '/node2': ['/topicB']}
        nodes_subscribe_topics = {}

        # config_tests = [(KeyError, {'topics_wrong_name': [['/topicA', '/topicB']], 'type': 'timing'}),
        #                 (KeyError, {'type': 'timing'}),
        #                 (KeyError, {}),
        #                 (TypeError, "not_a_dict"),
        #                 (TypeError, 1),
        #                 (ValueError, {'topics': [], 'type': 'timing'}),
        #                 (ValueError, {'topics': [[]], 'type': 'timing'}),
        #                 (ValueError, {'topics': [['']], 'type': 'timing'}),
        #                 (TypeError, {'topics': [[1]], 'type': 'timing'}),
        #                 (TypeError, {'topics': ["no_list_list"], 'type': 'timing'}),
        #                 (TypeError, {'topics': [1], 'type': 'timing'}),
        #                 (ValueError, {'topics': [['/wrong_topic_name', '/topicB']], 'type': 'timing'}),
        #                 (ValueError, {'topics': [], 'type': 'timing'}),
        #                 (TypeError, {'topics': [''], 'type': 'timing'}),
        #                 (TypeError, {'topics': [1], 'type': 'timing'}),
        #                 (TypeError, {'topics': "no_list", 'type': 'timing'}),
        #                 (TypeError, {'topics': 1, 'type': 'timing'}),
        #                 (TypeError, {'topics': ['/topic', '/topic2', '/topic3'], 'type': 'timing'})
        #                 ]
        config_tests = [(KeyError, observer_configuration(type="timing_wrong", resource=['/topicA', '/topicB'])),
                        (ValueError, observer_configuration(type="timing")),
                        (KeyError, observer_configuration()),
                        (TypeError, observer_configuration),
                        (TypeError, 1),
                        (ValueError, observer_configuration(type="timing", resource=[''])),
                        (ValueError, observer_configuration(type="timing", resource=[1])),
                        (TypeError, observer_configuration(type="timing", resource='no_list')),
                        (TypeError, observer_configuration(type="timing", resource=1)),
                        (ValueError, observer_configuration(type="timing", resource=['/topicA', '/topicB', '/topicC'])),
                        (ValueError, observer_configuration(type="timing", resource=['/topicA'])),
                        (ValueError, observer_configuration(type="timing", resource=['/topic_wrong']))
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                TimingObserver.generate_model_parameter(config,
                                                        topics_published_from_nodes, topics_subscribed_from_nodes,
                                                        nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):

        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
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

                TimingObserver.generate_model_parameter(config,
                                                        topics_published_from_nodes, topics_subscribed_from_nodes,
                                                        nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_3(self):
        # config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        config = observer_configuration(type="timing", resource=['/topicA', '/topicB'])
        topics_published_from_nodes = {'/topicA': ['node1'], '/topicB': ['node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'node1': ['/topicA'], 'node2': ['/topicB']}
        nodes_subscribe_topics = {}

        topics_subscribed_from_nodes_tests = \
            [
                # (ValueError, {'/topicA': ['node2']}, {'/node2': ['/topicA']}),
                (ValueError, {'/topicA': ['node2']}, {'node2': ['/']}),
                (ValueError, {'/topicA': ['node2']}, {'node2': ['']}),
                (TypeError, {'/topicA': ['node2']}, {'node2': [1]}),
                (ValueError, {'/topicA': ['node2']}, {'node2': ['/topicA', '/']}),
                (ValueError, {'/topicA': ['node2']}, {'node2': ['/topicA', '']}),
                (TypeError, {'/topicA': ['node2']}, {'node2': ['/topicA', 1]}),
                (TypeError, {'/topicA': ['node2']}, {'node2': 'not_a_list'}),
                (TypeError, {'/topicA': ['node2']}, {'node2': 1}),

                (ValueError, {'/topicA': ['/']}, {'node2': ['/topicA']}),
                (ValueError, {'/topicA': ['']}, {'node2': ['/topicA']}),
                (TypeError, {'/topicA': [1]}, {'node2': ['/topicA']}),
                (ValueError, {'/topicA': ['node2', '/']}, {'node2': ['/topicA']}),
                (ValueError, {'/topicA': ['node2', '']}, {'node2': ['/topicA']}),
                (TypeError, {'/topicA': ['node2', 1]}, {'node2': ['/topicA']}),
                (TypeError, {'/topicA': 'not_a_list'}, {'node2': ['/topicA']}),
                (TypeError, {'/topicA': 1}, {'node2': ['/topicA']}),
            ]

        for (error, topics_subscribed_from_nodes, nodes_subscribe_topics) in topics_subscribed_from_nodes_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(
                    topics_subscribed_from_nodes) + "' + '" + str(nodes_subscribe_topics) + "'",

                TimingObserver.generate_model_parameter(config,
                                                        topics_published_from_nodes, topics_subscribed_from_nodes,
                                                        nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(TimingObserver.decrypt_resource_info("/topicA [nodeA1, nodeA2] /topicB [nodeB1, nodeB2]"),
                         ['timing_obs_/topicA_nodeA1_/topicB_nodeB1',
                          'timing_obs_/topicA_nodeA1_/topicB_nodeB2',
                          'timing_obs_/topicA_nodeA2_/topicB_nodeB1',
                          'timing_obs_/topicA_nodeA2_/topicB_nodeB2'], "Topic name decryption not correct!")
        self.assertEqual(TimingObserver.decrypt_resource_info("/topicA [] /topicB []"),
                         ['timing_obs_/topicA_all_/topicB_all'], "Topic name decryption not correct!")
        self.assertEqual(TimingObserver.decrypt_resource_info("/topicA [node1]/topicB []"),
                         ['timing_obs_/topicA_node1_/topicB_all'], "Topic name decryption not correct!")
        self.assertEqual(TimingObserver.decrypt_resource_info("/topicA [node1,node2, node3]/topicB []"),
                         ['timing_obs_/topicA_node1_/topicB_all',
                          'timing_obs_/topicA_node2_/topicB_all',
                          'timing_obs_/topicA_node3_/topicB_all'], "Topic name decryption not correct!")
        resource_info_tests = [
            (ValueError, "/ [nodeA1, nodeA2] /topicB []"),
            (ValueError, "/ [nodeA1, nodeA2] /topicB [nodeB1]"),
            (ValueError, "/ [nodeA1, nodeA2] /topicB [nodeB1, nodeB2]"),
            (ValueError, "/ [nodeA1, ] /topicB []"),
            (ValueError, "/ [nodeA1, ] /topicB [nodeB1]"),
            (ValueError, "/ [nodeA1, ] /topicB [nodeB1, nodeB2]"),
            (ValueError, "/ [] /topicB []"),
            (ValueError, "/ [] /topicB [nodeB1]"),
            (ValueError, "/ [] /topicB [nodeB1, nodeB2]"),

            (ValueError, "/topicA [] / []"),
            (ValueError, "/topicA [] / [nodeB1]"),
            (ValueError, "/topicA [] / [nodeB1, nodeB2]"),
            (ValueError, "/topicA [nodeA1] / []"),
            (ValueError, "/topicA [nodeA1] / [nodeB1]"),
            (ValueError, "/topicA [nodeA1] / [nodeB1, nodeB2]"),
            (ValueError, "/topicA [nodeA1, nodeA2] / []"),
            (ValueError, "/topicA [nodeA1, nodeA2] / [nodeB1]"),
            (ValueError, "/topicA [nodeA1, nodeA2] / [nodeB1, nodeB2]"),

            (ValueError, "/ [] / []"),
            (ValueError, "/ [] / [nodeB1]"),
            (ValueError, "/ [] / [nodeB1, nodeB2]"),
            (ValueError, "/ [nodeA1] / []"),
            (ValueError, "/ [nodeA1] / [nodeB1]"),
            (ValueError, "/ [nodeA1] / [nodeB1, nodeB2]"),
            (ValueError, "/ [nodeA1, nodeA2] / []"),
            (ValueError, "/ [nodeA1, nodeA2] / [nodeB1]"),
            (ValueError, "/ [nodeA1, nodeA2] / [nodeB1, nodeB2]"),

            (ValueError, "/[]/[]"),
            (ValueError, "/[]/topicB[nodeB1]"),
            (ValueError, "/[]/topicB[nodeB1, nodeB2]"),
            (ValueError, "/topicA[]/[nodeB1]"),
            (ValueError, "/topicA[]/[nodeB1, nodeB2]"),

            (ValueError, "/topicA [/] /topicB []"),
            (ValueError, "/topicA [/] /topicB [/nodeB1]"),
            (ValueError, "/topicA [/] /topicB [/nodeB1, nodeB2]"),

            (ValueError, "/topicA [/nodeA1,] /topicB []"),
            (ValueError, "/topicA [/nodeA1,] /topicB [/nodeB1]"),
            (ValueError, "/topicA [/nodeA1,] /topicB [/nodeB1, nodeB2]"),

            (ValueError, "/topicA [/nodeA1, /] /topicB []"),
            (ValueError, "/topicA [/nodeA1, /] /topicB [/nodeB1]"),
            (ValueError, "/topicA [/nodeA1, /] /topicB [/nodeB1, nodeB2]"),

            (ValueError, "/topicA [] /topicB [/]"),
            (ValueError, "/topicA [nodeA1] /topicB [/]"),
            (ValueError, "/topicA [nodeA1,nodeA2] /topicB [/]"),

            (ValueError, "/topicA [] /topicB [/nodeB1,]"),
            (ValueError, "/topicA [/nodeA1] /topicB [/nodeB1,]"),
            (ValueError, "/topicA [/nodeA1,nodeA2] /topicB [/nodeB1,]"),

            (ValueError, "/topicA [/] /topicB [/]"),
            (ValueError, "/topicA [/nodeA1, /] /topicB [/nodeB1, /]"),
            (ValueError, "/topicA [/nodeA1, ] /topicB [/nodeB1, ]"),

            (TypeError, 1),
        ]

        for (error, resource_info) in resource_info_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(resource_info) + "'",
                TimingObserver.decrypt_resource_info(resource_info)
            print "... DONE"
