from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from base_observer import *


class TimestampObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, topic):
        BaseObserver.__init__(self)
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(topic)

        self.ab_node = ab_node
        self.topic = topic
        
    def __repr__(self):
        return "(!%s => %s)" % (self.ab_node, self.topic)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.topic)]

    @staticmethod
    def generate_model_parameter(config, topics_from_nodes):
        checkInputData.dict_data_valid(config, False)
        topics = config['topics']

        checkInputData.list_data_valid(topics)

        checkInputData.dict_data_valid(topics_from_nodes, False)

        vars = {}
        rules = []
        nodes = []

        for topic in topics:
            checkInputData.list_data_valid(topics_from_nodes[topic])

            for callerid in topics_from_nodes[topic]:

                observation = "timestamp_obs_" + topic + "_" + callerid
                vars[observation] = Variable(observation, Variable.BOOLEAN, None)
                rules.append(TimestampObserver(ab_pred(str(callerid)), observation))

            new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter("timestamp", topic, topics_from_nodes[topic])
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


import unittest


class TestTimestampObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_timestamp_observer(self):
        with self.assertRaises(ValueError):
            TimestampObserver(ab_pred(""), "/topic")
        with self.assertRaises(ValueError):
            TimestampObserver(ab_pred("/"), "/topic")
        with self.assertRaises(ValueError):
            TimestampObserver("", "/topic")
        with self.assertRaises(ValueError):
            TimestampObserver("/", "/topic")
        with self.assertRaises(ValueError):
            TimestampObserver(ab_pred("name"), "")
        with self.assertRaises(ValueError):
            TimestampObserver(ab_pred(""), "")
        with self.assertRaises(ValueError):
            TimestampObserver("", "")
        with self.assertRaises(TypeError):
            TimestampObserver(1, "/topic")
        with self.assertRaises(TypeError):
            TimestampObserver(ab_pred("name"), 1)
        with self.assertRaises(ValueError):
            TimestampObserver(ab_pred("name"), "/")

    def test_clause(self):
        observer = TimestampObserver(ab_pred("name"), "/topic")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")

    def test_generate_model_parameter(self):
        config = {'topics': ['/topic'], 'type': 'timestamp'}
        topics_from_nodes = {'/topic': ['/node1', '/node2']}
        vars, rules, nodes = TimestampObserver.generate_model_parameter(config, topics_from_nodes)

        vars_req = {'timestamp_obs_/topic_all': Variable('timestamp_obs_/topic_all', 1, None),
                    'timestamp_obs_/topic_/node1': Variable('timestamp_obs_/topic_/node1', 1, None),
                    'timestamp_obs_/topic_/node2': Variable('timestamp_obs_/topic_/node2', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "timestamp added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [TimestampObserver(ab_pred('/node1'), 'timestamp_obs_/topic_/node1'),
                     TimestampObserver(ab_pred('/node2'), 'timestamp_obs_/topic_/node2'),
                     CalleridsObserver('timestamp_obs_/topic_all', ['timestamp_obs_/topic_/node1', 'timestamp_obs_/topic_/node2'])]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Timestamp added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Timestamp should not add nodes!")

        # test different arguments for the config-parameter which all should raise exeptions
        topics_from_nodes = {'/topic': ['/node1', '/node2']}
        with self.assertRaises(KeyError):
            TimestampObserver.generate_model_parameter({'topics_wrong_name': ['/topic'], 'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            TimestampObserver.generate_model_parameter({'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            TimestampObserver.generate_model_parameter({}, topics_from_nodes)
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter("not_a_dict", topics_from_nodes)
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter(1, topics_from_nodes)
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter({'topics': [], 'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter({'topics': [''], 'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter({'topics': [1], 'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter({'topics': "no_list", 'type': 'timestamp'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter({'topics': 1, 'type': 'timestamp'}, topics_from_nodes)

        # test different arguments for the topics_from_nodes-parameter which all should raise exeptions
        config = {'topics': ['/topic'], 'type': 'timestamp'}
        with self.assertRaises(KeyError):
            TimestampObserver.generate_model_parameter(config, {'/topic_wrong_name': ['/node1', '/node2']})
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter(config, {'/topic': []})
        with self.assertRaises(KeyError):
            TimestampObserver.generate_model_parameter(config, {})
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter(config, "no_dict")
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter(config, 1)
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter(config, {'/topic': ['/', '/node2']})
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter(config, {'/topic': ['', '/node2']})
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter(config, {'/topic': [1, '/node2']})
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter(config, {'/topic': ['/node1', '/']})
        with self.assertRaises(ValueError):
            TimestampObserver.generate_model_parameter(config, {'/topic': ['/node1', '']})
        with self.assertRaises(TypeError):
            TimestampObserver.generate_model_parameter(config, {'/topic': ['/node1', 1]})

    def test_decrypt_resource_info(self):
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1, node2]"), ['timestamp_obs_/topic_name_node1', 'timestamp_obs_/topic_name_node2'], "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1, node2, node3, node4, node5]"), ['timestamp_obs_/topic_name_node1', 'timestamp_obs_/topic_name_node2', 'timestamp_obs_/topic_name_node3',
                                                                                                               'timestamp_obs_/topic_name_node4', 'timestamp_obs_/topic_name_node5'], "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name [node1]"), ['timestamp_obs_/topic_name_node1'], "Topic name decryption not correct!")
        self.assertEqual(TimestampObserver.decrypt_resource_info("/topic_name {}"), ['timestamp_obs_/topic_name_all'], "Topic name decryption not correct!")

        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/ [node1, node2]")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/ [node1, /]")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/topic_name")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/topic_name [/]")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/topic_name [/node1, ]")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/topic_name [/node1, /]")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("/")
        with self.assertRaises(ValueError):
            TimestampObserver.decrypt_resource_info("")
        with self.assertRaises(TypeError):
            TimestampObserver.decrypt_resource_info(1)