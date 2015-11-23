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
    
    def __init__(self, ab_node_a, ab_node_b, ab_function, observation):
        super(MovementObserver, self).__init__()
        checkInputData.list_data_valid(ab_node_a)
        checkInputData.list_data_valid(ab_node_b)
        checkInputData.str_data_valid(ab_function)
        checkInputData.str_data_valid(observation)

        self.ab_node_a = ab_node_a
        self.ab_node_b = ab_node_b
        self.ab_function = ab_function
        self.observation = observation

    def __repr__(self):
        return "(!%s && !%s && !%s => %s)" % (self.ab_node_a, self.ab_node_b, self.ab_function, self.observation)

    def to_clause(self):
        return [clause(all_pos(self.ab_node_a) + " " + all_pos(self.ab_node_b) + " " + self.ab_function + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes={}):
        checkInputData.dict_data_valid(config, check_entries=False)
        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False)
        topics = config['topics']

        vars = {}
        rules = []
        nodes = []

        checkInputData.list_data_valid(topics, check_entries=False)

        nodes.append("movement")
        vars["movement"] = Variable("movement", Variable.BOOLEAN, None)
        vars[ab_pred("movement")] = Variable(ab_pred("movement"), Variable.BOOLEAN, None)

        for topic_pair in topics:
            checkInputData.list_data_valid(topic_pair, check_entries=True)
            topic_a = topic_pair[0]
            topic_b = topic_pair[1]

            observation = "movement_obs_" + topic_a + "_" + topic_b
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)

            checkInputData.list_data_valid(topics_published_from_nodes[topic_a], check_entries=True)
            checkInputData.list_data_valid(topics_published_from_nodes[topic_b], check_entries=True)
            rules.append(MovementObserver(all_ab_pred(topics_published_from_nodes[topic_a]), all_ab_pred(topics_published_from_nodes[topic_b]), ab_pred("movement"), observation))

        return vars, rules, nodes

    @staticmethod
    def decrypt_resource_info(resource_info):
        if not resource_info:
            raise ValueError
        if not isinstance(resource_info, str):
            raise TypeError

        entries = re.findall(regex_prog, resource_info)
        if not len(entries):
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

    def test_movement_observer(self):
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("")], [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("/")], [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([""], [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver(["/"], [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([1], [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver("/", [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver("node1", [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver("", [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver(1, [ab_pred("topic2")], ab_pred("movement"), "movement_obs_node1_node2")

        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("topic1")], [ab_pred("")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("topic1")], [ab_pred("/")], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("topic1")], [""], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("topic1")], ["/"], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("topic1")], [1], ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("topic1")], "/", ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("topic1")], "node2", ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("topic1")], "", ab_pred("movement"), "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("topic1")], 1, ab_pred("movement"), "movement_obs_node1_node2")

        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred(""), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("/"), "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver(["node1"], [ab_pred("node2")], "", "movement_obs_node1_node2")
        with self.assertRaises(ValueError):
            MovementObserver(["/node1"], [ab_pred("node2")], "/", "movement_obs_node1_node2")
        with self.assertRaises(TypeError):
            MovementObserver([ab_pred("node1")], [ab_pred("node2")], 1, "movement_obs_node1_node2")

        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("movement"), "")
        with self.assertRaises(ValueError):
            MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("movement"), "/")
        with self.assertRaises(TypeError):
            MovementObserver(["node1"], [ab_pred("node2")], ab_pred("movement"), 1)


    def test_clause(self):
        observer = MovementObserver([ab_pred("node1")], [ab_pred("node2")], ab_pred("movement"), "movement_obs_node1_node2")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node1"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("node2"), "Second literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("movement"), "Third literal in clause does not match! ")
        print new_clause.literals[3]
        self.assertEqual(str(new_clause.literals[3]), "movement_obs_node1_node2", "Fourth literal in clause does not match!")

    def test_generate_model_parameter(self):
        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
        topics_published_from_nodes = {'/topicB': ['nodeB'], '/topicA': ['nodeA', 'nodeC']}
        vars, rules, nodes = MovementObserver.generate_model_parameter(config, topics_published_from_nodes)

        vars_req = {'movement': Variable('movement', 1, None),
                    ab_pred("movement"): Variable(ab_pred("movement"), 1, None),
                    'movement_obs_/topicA_/topicB': Variable('movement_obs_/topicA_/topicB', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "Movement added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [MovementObserver([ab_pred('nodeA'), ab_pred('nodeC')], [ab_pred('nodeB')], ab_pred("movement"), "movement_obs_/topicA_/topicB")]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Movement added wrong number of rules!")


        self.assertEqual(len(nodes), 1, "Movement should add one node!")
        self.assertTrue("movement" in nodes, "'movement' not in nodes-list!")

        # test different arguments for the config-parameter which all should raise exeptions
        topics_published_from_nodes = {'/topicB': ['nodeB'], '/topicA': ['nodeA', 'nodeC']}
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter({'topics_wrong_name': [['/topicA', '/topicB']], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter({'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter({}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter("not_a_dict", topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(1, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': [1], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': [""], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': ["/"], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': ["no_list"], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter({'topics': [], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter({'topics': [[]], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter({'topics': [['']], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': [[1]], 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': "no_list_list", 'type': 'movement'}, topics_published_from_nodes)
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter({'topics': 1, 'type': 'movement'}, topics_published_from_nodes)

        # test different arguments for the topics_from_nodes-parameter which all should raise exeptions
        config = {'topics': [['/topicA', '/topicB']], 'type': 'movement'}
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topic_wrong_name': ['/node1', '/node2']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': []})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {})
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, "no_dict")
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, 1)
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/', '/node2']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['', '/node2']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': [1, '/node2']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', '/']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', '']})
        with self.assertRaises(KeyError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', 1]})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/', '/node2'], '/topicA': ['/nodeA1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['', '/node2'], '/topicA': ['/nodeA1']})
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, {'/topicB': [1, '/node2'], '/topicA': ['/nodeA1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', '/'], '/topicA': ['/nodeA1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', ''], '/topicA': ['/nodeA1']})
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node1', 1], '/topicA': ['/nodeA1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/', '/node1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['', '/node1']})
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': [1, '/node1']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', '/']})
        with self.assertRaises(ValueError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', '']})
        with self.assertRaises(TypeError):
            MovementObserver.generate_model_parameter(config, {'/topicB': ['/node2'], '/topicA': ['/node1', 1]})

    def test_decrypt_resource_info(self):
        self.assertEqual(MovementObserver.decrypt_resource_info("/topic_1 /topic_2"), ['movement_obs_/topic_1_/topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("topic_1 /topic_2"), ['movement_obs_topic_1_/topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("/topic_1 topic_2"), ['movement_obs_/topic_1_topic_2'], "Topic name decryption not correct!")
        self.assertEqual(MovementObserver.decrypt_resource_info("topic_1 topic_2"), ['movement_obs_topic_1_topic_2'], "Topic name decryption not correct!")

        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("/ /topic_2")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("/topic_1_ /")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("/ /")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info(" /topic_name")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("/topic_name ")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("/")
        with self.assertRaises(ValueError):
            MovementObserver.decrypt_resource_info("")
        with self.assertRaises(TypeError):
            MovementObserver.decrypt_resource_info(1)

