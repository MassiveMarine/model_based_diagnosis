from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class HzObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """
    def __init__(self, ab_node, topic):
        super(HzObserver, self).__init__()
        if not ab_node or ab_node is ab_pred(""):
            raise ValueError
        if not isinstance(ab_node, str):
            raise TypeError
        if not topic or topic is "/":
            raise ValueError
        if not isinstance(topic, str):
            raise TypeError

        self.ab_node = ab_node
        self.topic = topic

    def __repr__(self):
        return "hz: %s, %s" % (self.ab_node, self.topic)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.topic)]

    @staticmethod
    def generate_model_parameter(config, topics_from_nodes):
        if not len(config['topics']):
            raise ValueError
        if not isinstance(config['topics'], list):
            raise TypeError

        vars = {}
        rules = []
        nodes = []

        for topic in config['topics']:
            if topic is '':
                raise ValueError
            if not isinstance(topic, str):
                raise TypeError
            if not len(topics_from_nodes[topic]):
                raise ValueError
            if not isinstance(topics_from_nodes[topic], list):
                raise TypeError

            print topics_from_nodes[topic]
            for callerid in topics_from_nodes[topic]:
                if callerid is '/' or callerid is '':
                    raise ValueError
                if not isinstance(callerid, str):
                    raise TypeError

                observation = "hz_obs_" + topic + "_" + callerid
                vars[observation] = Variable(observation, Variable.BOOLEAN, None)
                rules.append(HzObserver(ab_pred(str(callerid)), observation))

            new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter("hz", topic, topics_from_nodes[topic])
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

        if not topic_name or topic_name is "/":
            raise ValueError
        if not isinstance(topic_name, str):
            raise TypeError

        if len(callerids_str) <= 2:
            return ['hz_obs_' + str(topic_name) + "_all"]

        callerids = [x.strip() for x in callerids_str[1:-1].split(',')]

        infos = []
        for callerid in callerids:
            infos.append('hz_obs_' + str(topic_name) + "_" + str(callerid))

        return infos


picosat.SENTENCE_INTERPRETERS[HzObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['hz'] = HzObserver


import unittest


class TestHzObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_hz_observer(self):
        with self.assertRaises(ValueError):
            HzObserver(ab_pred(""), "/topic")
        with self.assertRaises(ValueError):
            HzObserver("", "/topic")
        with self.assertRaises(ValueError):
            HzObserver(ab_pred("name"), "")
        with self.assertRaises(ValueError):
            HzObserver(ab_pred(""), "")
        with self.assertRaises(ValueError):
            HzObserver("", "")
        with self.assertRaises(TypeError):
            HzObserver(1, "/topic")
        with self.assertRaises(TypeError):
            HzObserver(ab_pred("name"), 1)
        with self.assertRaises(ValueError):
            HzObserver(ab_pred("name"), "/")

    def test_clause(self):
        observer = HzObserver(ab_pred("name"), "/topic")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")

    def test_generate_model_parameter(self):
        config = {'topics': ['/topic'], 'type': 'hz'}
        topics_from_nodes = {'/topic': ['/node1', '/node2']}
        vars, rules, nodes = HzObserver.generate_model_parameter(config, topics_from_nodes)

        vars_req = {'hz_obs_/topic_all': Variable('hz_obs_/topic_all', 1, None),
                    'hz_obs_/topic_/node1': Variable('hz_obs_/topic_/node1', 1, None),
                    'hz_obs_/topic_/node2': Variable('hz_obs_/topic_/node2', 1, None)}

        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [HzObserver(ab_pred('/node1'), 'hz_obs_/topic_/node1'),
                     HzObserver(ab_pred('/node2'), 'hz_obs_/topic_/node2'),
                     CalleridsObserver('hz_obs_/topic_all', ['hz_obs_/topic_/node1', 'hz_obs_/topic_/node2'])]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Hz added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "Hz should not add nodes!")

        # test different arguments for the config-parameter which all should raise exeptions
        topics_from_nodes = {'/topic': ['/node1', '/node2']}
        with self.assertRaises(KeyError):
            HzObserver.generate_model_parameter({'topics_wrong_name': ['/topic'], 'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            HzObserver.generate_model_parameter({'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            HzObserver.generate_model_parameter({}, topics_from_nodes)
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter("not_a_dict", topics_from_nodes)
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter(1, topics_from_nodes)
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter({'topics': [], 'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter({'topics': [''], 'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter({'topics': [1], 'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter({'topics': "no_list", 'type': 'hz'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter({'topics': 1, 'type': 'hz'}, topics_from_nodes)

        # test different arguments for the topics_from_nodes-parameter which all should raise exeptions
        config = {'topics': ['/topic'], 'type': 'hz'}
        with self.assertRaises(KeyError):
            HzObserver.generate_model_parameter(config, {'/topic_wrong_name': ['/node1', '/node2']})
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter(config, {'/topic': []})
        with self.assertRaises(KeyError):
            HzObserver.generate_model_parameter(config, {})
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter(config, "no_dict")
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter(config, 1)
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter(config, {'/topic': ['/', '/node2']})
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter(config, {'/topic': ['', '/node2']})
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter(config, {'/topic': [1, '/node2']})
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter(config, {'/topic': ['/node1', '/']})
        with self.assertRaises(ValueError):
            HzObserver.generate_model_parameter(config, {'/topic': ['/node1', '']})
        with self.assertRaises(TypeError):
            HzObserver.generate_model_parameter(config, {'/topic': ['/node1', 1]})

    def test_decrypt_resource_info(self):
        self.assertEqual(HzObserver.decrypt_resource_info("/topic_name [node1, node2]"), ['hz_obs_/topic_name_node1', 'hz_obs_/topic_name_node2'], "Topic name decryption not correct!")
        self.assertEqual(HzObserver.decrypt_resource_info("/topic_name [node1, node2, node3, node4, node5]"), ['hz_obs_/topic_name_node1', 'hz_obs_/topic_name_node2', 'hz_obs_/topic_name_node3',
                                                                                                               'hz_obs_/topic_name_node4', 'hz_obs_/topic_name_node5'], "Topic name decryption not correct!")
        self.assertEqual(HzObserver.decrypt_resource_info("/topic_name [node1]"), ['hz_obs_/topic_name_node1'], "Topic name decryption not correct!")
        self.assertEqual(HzObserver.decrypt_resource_info("/topic_name {}"), ['hz_obs_/topic_name_all'], "Topic name decryption not correct!")

        with self.assertRaises(ValueError):
            HzObserver.decrypt_resource_info("/ [node1, node2]")
        with self.assertRaises(ValueError):
            HzObserver.decrypt_resource_info("/topic_name")
        with self.assertRaises(ValueError):
            HzObserver.decrypt_resource_info("/")
        with self.assertRaises(ValueError):
            HzObserver.decrypt_resource_info("")
        with self.assertRaises(TypeError):
            HzObserver.decrypt_resource_info(1)

