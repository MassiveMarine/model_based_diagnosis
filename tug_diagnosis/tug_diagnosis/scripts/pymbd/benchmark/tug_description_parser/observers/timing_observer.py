from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *

import re
regex_prog = re.compile('(\S*)\s?(\[\S*\s?\S*\s?\])\s?(\S*)\s?(\[\S*\s?\S*\s?\])')


class TimingObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_nodes, topic):
        super(TimingObserver, self).__init__()
        checkInputData.list_data_valid(ab_nodes, check_entries=True)
        checkInputData.str_data_valid(topic)

        self.ab_nodes = ab_nodes
        self.topic = topic

    def __repr__(self):
        return "(!%s => %s)" % (self.ab_nodes, self.topic)

    def to_clause(self):
        return [clause(all_pos(self.ab_nodes) + " " + self.topic)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes):
        print '=========='
        checkInputData.dict_data_valid(config, check_entries=False)
        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False)
        checkInputData.dict_data_valid(topics_subscribed_from_nodes, check_entries=False, allow_empty=True)
        topics = config['topics']

        vars = {}
        rules = []
        nodes = []

        checkInputData.list_data_valid(topics, check_entries=False)

        for topic_pair in topics:
            checkInputData.list_data_valid(topic_pair, check_entries=True)
            topicA = topic_pair[0]
            topicB = topic_pair[1]

            nodes_list = []
            checkInputData.list_data_valid(topics_published_from_nodes[topicB], check_entries=True)
            for node in topics_published_from_nodes[topicB]:
                if node in topics_subscribed_from_nodes.get(topicA, []):
                    nodes_list += topics_published_from_nodes[topicB]
                    break

            checkInputData.list_data_valid(topics_published_from_nodes[topicA], check_entries=True)
            for node in topics_published_from_nodes[topicA]:
                if node in topics_subscribed_from_nodes.get(topicB, []):
                    nodes_list += topics_published_from_nodes[topicA]

            if not nodes_list:
                nodes_list = topics_published_from_nodes[topicA] + topics_published_from_nodes[topicB]

            for calleridA in topics_published_from_nodes[topicA]:
                for calleridB in topics_published_from_nodes[topicB]:

                    observation = "timing_obs_" + topicA + "_" + calleridA + "_" + topicB + "_" + calleridB
                    vars[observation] = Variable(observation, Variable.BOOLEAN, None)

                    rules.append(TimingObserver(all_ab_pred(nodes_list), observation))

            new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter_2("timing", topicA, topics_published_from_nodes[topicA], topicB, topics_published_from_nodes[topicB] )
            vars.update(new_vars)
            rules += new_rules
            nodes += new_nodes

        return vars, rules, nodes

    @staticmethod
    def decrypt_resource_info(resource_info):

        if not resource_info:
            raise ValueError
        if not isinstance(resource_info, str):
            raise TypeError

        [topicA_name, calleridsA_str, topicB_name, calleridsB_str] = list(re.findall(regex_prog,resource_info)[0])
        # [topicA_name, calleridsA_str, topicB_name, calleridsB_str] = resource_info.split(' ')

        print topicA_name
        print calleridsA_str
        print topicB_name
        print calleridsB_str

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
                infos.append('timing_obs_' + str(topicA_name) + "_" + str(calleridA) + "_" + str(topicB_name) + "_" + str(calleridB))

        print infos
        return infos


picosat.SENTENCE_INTERPRETERS[TimingObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timing'] = TimingObserver


import unittest


class TestTimingObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_timing_observer(self):
        with self.assertRaises(ValueError):
            TimingObserver([ab_pred("")], "/topic")
        with self.assertRaises(ValueError):
            TimingObserver([ab_pred("/")], "/topic")
        with self.assertRaises(ValueError):
            TimingObserver([""], "/topic")
        with self.assertRaises(ValueError):
            TimingObserver(["/"], "/topic")
        with self.assertRaises(TypeError):
            TimingObserver([1], "/topic")
        with self.assertRaises(TypeError):
            TimingObserver(ab_pred(""), "/topic")
        with self.assertRaises(TypeError):
            TimingObserver(ab_pred("/"), "/topic")
        with self.assertRaises(TypeError):
            TimingObserver("", "/topic")
        with self.assertRaises(TypeError):
            TimingObserver("/", "/topic")
        with self.assertRaises(TypeError):
            TimingObserver(1, "/topic")
        with self.assertRaises(TypeError):
            TimingObserver([["/node1"]], "/topic")


        with self.assertRaises(ValueError):
            TimingObserver([ab_pred("name")], "")
        with self.assertRaises(ValueError):
            TimingObserver([ab_pred("")], "")
        with self.assertRaises(ValueError):
            TimingObserver([""], "")
        with self.assertRaises(TypeError):
            TimingObserver([ab_pred("name")], 1)
        with self.assertRaises(ValueError):
            TimingObserver([ab_pred("name")], "/")

    def test_clause(self):
        observer = TimingObserver([ab_pred("name1")], "/topic")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")

        observer = TimingObserver([ab_pred("name1"), ab_pred("name2")], "/topic")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name1"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("name2"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), "/topic", "Third literal in clause does not match!")

    def test_generate_model_parameter(self):

        # +----------+ /topicA  +---------+ /topicB
        # | starter1 |--------->| timing1 |------->
        # +----------+          +---------+
        config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        topics_published_from_nodes = {'/topicB': ['timing1'], '/topicA': ['starter1']}
        topics_subscribed_from_nodes = {'/topicA': ['timing1']}
        vars, rules, nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes)

        vars_req = {'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_timing1': Variable('timing_obs_/topicA_all_/topicB_timing1', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_all': Variable('timing_obs_/topicA_starter1_/topicB_all', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_timing1': Variable('timing_obs_/topicA_starter1_/topicB_timing1', 1, None)}


        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")
        rules_req = [TimingObserver(all_ab_pred(['timing1']), 'timing_obs_/topicA_starter1_/topicB_timing1'),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_timing1']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_all']),
                     CalleridsObserver('timing_obs_/topicA_starter1_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_timing1']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_timing1', ['timing_obs_/topicA_starter1_/topicB_timing1']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")

        # +----------+ /topicA
        # | starter1 |--------->
        # +----------+
        #
        # +----------+ /topicB
        # | starter2 |--------->
        # +----------+
        config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        topics_published_from_nodes = {'/topicB': ['starter2'], '/topicA': ['starter1']}
        topics_subscribed_from_nodes = {}
        vars, rules, nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes)

        vars_req = {}
        vars_req = {'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_starter2': Variable('timing_obs_/topicA_all_/topicB_starter2', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_all': Variable('timing_obs_/topicA_starter1_/topicB_all', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_starter2': Variable('timing_obs_/topicA_starter1_/topicB_starter2', 1, None)}


        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")
        rules_req = [TimingObserver(all_ab_pred(['starter1', 'starter2']), 'timing_obs_/topicA_starter1_/topicB_starter2'),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_starter2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_all']),
                     CalleridsObserver('timing_obs_/topicA_starter1_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_starter2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_starter2', ['timing_obs_/topicA_starter1_/topicB_starter2']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")

        # +----------+ /topicA         +---------+ /topicB
        # | starter1 |----------+  +-->| timing1 |----------+
        # +----------+          |  |   +---------+          |
        #                       *--*                        +--->
        # +----------+ /topicA  |  |   +---------+ /topicB  |
        # | starter2 |---------->  +-->| timing2 |----------+
        # +----------+                 +---------+
        config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        topics_published_from_nodes = {'/topicB': ['timing1', 'timing2'], '/topicA': ['starter1', 'starter2']}
        topics_subscribed_from_nodes = {'/topicA': ['timing1', 'timing2']}
        vars, rules, nodes = TimingObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes)

        vars_req = {}
        vars_req = {'timing_obs_/topicA_starter1_/topicB_all': Variable('timing_obs_/topicA_starter1_/topicB_all', 1, None),
                    'timing_obs_/topicA_starter2_/topicB_timing2': Variable('timing_obs_/topicA_starter2_/topicB_timing2', 1, None),
                    'timing_obs_/topicA_starter2_/topicB_all': Variable('timing_obs_/topicA_starter2_/topicB_all', 1, None),
                    'timing_obs_/topicA_starter2_/topicB_timing1': Variable('timing_obs_/topicA_starter2_/topicB_timing1', 1, None),
                    'timing_obs_/topicA_all_/topicB_all': Variable('timing_obs_/topicA_all_/topicB_all', 1, None),
                    'timing_obs_/topicA_all_/topicB_timing2': Variable('timing_obs_/topicA_all_/topicB_timing2', 1, None),
                    'timing_obs_/topicA_all_/topicB_timing1': Variable('timing_obs_/topicA_all_/topicB_timing1', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_timing1': Variable('timing_obs_/topicA_starter1_/topicB_timing1', 1, None),
                    'timing_obs_/topicA_starter1_/topicB_timing2': Variable('timing_obs_/topicA_starter1_/topicB_timing2', 1, None),
                    }


        self.assertEqual(len(vars), len(vars_req), "Timing added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")
        rules_req = [TimingObserver(all_ab_pred(['timing1', 'timing2']), 'timing_obs_/topicA_starter1_/topicB_timing1'),
                     TimingObserver(all_ab_pred(['timing1', 'timing2']), 'timing_obs_/topicA_starter1_/topicB_timing2'),
                     TimingObserver(all_ab_pred(['timing1', 'timing2']), 'timing_obs_/topicA_starter2_/topicB_timing1'),
                     TimingObserver(all_ab_pred(['timing1', 'timing2']), 'timing_obs_/topicA_starter2_/topicB_timing2'),

                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_all_/topicB_timing1', 'timing_obs_/topicA_all_/topicB_timing2']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_all', 'timing_obs_/topicA_starter2_/topicB_all']),

                     CalleridsObserver('timing_obs_/topicA_starter1_/topicB_all', ['timing_obs_/topicA_starter1_/topicB_timing1', 'timing_obs_/topicA_starter1_/topicB_timing2']),
                     CalleridsObserver('timing_obs_/topicA_starter2_/topicB_all', ['timing_obs_/topicA_starter2_/topicB_timing1', 'timing_obs_/topicA_starter2_/topicB_timing2']),

                     CalleridsObserver('timing_obs_/topicA_all_/topicB_timing1', ['timing_obs_/topicA_starter1_/topicB_timing1', 'timing_obs_/topicA_starter2_/topicB_timing1']),
                     CalleridsObserver('timing_obs_/topicA_all_/topicB_timing2', ['timing_obs_/topicA_starter1_/topicB_timing2', 'timing_obs_/topicA_starter2_/topicB_timing2']),
                     ]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timing added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timing should not add nodes!")

        # test different arguments for the config-parameter which all should raise exeptions
        config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        topics_published_from_nodes = {'/topicB': ['timing1'], '/topicA': ['starter1']}
        topics_subscribed_from_nodes = {'/topicA': ['timing1']}
        with self.assertRaises(KeyError):
            TimingObserver.generate_model_parameter({'topics_wrong_name': [['/topicA', '/topicB']], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(KeyError):
            TimingObserver.generate_model_parameter({'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(KeyError):
            TimingObserver.generate_model_parameter({}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter("not_a_dict", topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(1, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': [1], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': [""], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': ["/"], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': ["no_list"], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter({'topics': [], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter({'topics': [[]], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter({'topics': [['']], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': [[1]], 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': "no_list_list", 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter({'topics': 1, 'type': 'timing'}, topics_published_from_nodes, topics_subscribed_from_nodes)

        # test different arguments for the topics_from_nodes-parameter which all should raise exeptions
        config = {'topics': [['/topicA', '/topicB']], 'type': 'timing'}
        # topics_published_from_nodes = {'/topicB': ['timing1'], '/topicA': ['starter1']}
        topics_subscribed_from_nodes = {'/topicA': ['timing1']}
        with self.assertRaises(KeyError):
            TimingObserver.generate_model_parameter(config, {'/topic_wrong_name': ['/node1', '/node2']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': []}, topics_subscribed_from_nodes)
        with self.assertRaises(KeyError):
            TimingObserver.generate_model_parameter(config, {}, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, "no_dict", topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, 1, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/', '/node2']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['', '/node2']}, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, {'/topicB': [1, '/node2']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node1', '/']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node1', '']}, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node1', 1]}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/', '/node1']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['', '/node1']}, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': [1, '/node1']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', '/']}, topics_subscribed_from_nodes)
        with self.assertRaises(ValueError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', '']}, topics_subscribed_from_nodes)
        with self.assertRaises(TypeError):
            TimingObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', 1]}, topics_subscribed_from_nodes)

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

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] /topicB [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] /topicB [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, ] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, ] /topicB [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, ] /topicB [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] /topicB [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] /topicB [nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [] / [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1] / [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1, nodeA2] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1, nodeA2] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1, nodeA2] / [nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [] / [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1] / [nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] / []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] / [nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/ [nodeA1, nodeA2] / [nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/[]/[]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/[]/topicB[nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/[]/topicB[nodeB1, nodeB2]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA[]/[nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA[]/[nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/] /topicB [/nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/] /topicB [/nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1,] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1,] /topicB [/nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1,] /topicB [/nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1, /] /topicB []")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1, /] /topicB [/nodeB1]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1, /] /topicB [/nodeB1, nodeB2]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [] /topicB [/]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1] /topicB [/]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [nodeA1,nodeA2] /topicB [/]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [] /topicB [/nodeB1,]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1] /topicB [/nodeB1,]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1,nodeA2] /topicB [/nodeB1,]")

        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/] /topicB [/]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1, /] /topicB [/nodeB1, /]")
        with self.assertRaises(ValueError):
            TimingObserver.decrypt_resource_info("/topicA [/nodeA1, ] /topicB [/nodeB1, ]")

        with self.assertRaises(TypeError):
            TimingObserver.decrypt_resource_info(1)

