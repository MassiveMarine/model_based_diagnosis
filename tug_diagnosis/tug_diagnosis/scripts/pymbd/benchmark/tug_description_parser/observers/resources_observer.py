from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class ResourcesObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, node):
        super(ResourcesObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(node)

        self.ab_node = ab_node
        self.node = node
        
    def __repr__(self):
        return "resources: %s, %s" % (self.ab_node, self.node)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.node)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes={}):
        checkInputData.dict_data_valid(config, False)
        nodes = config['nodes']
        checkInputData.list_data_valid(nodes)

        vars = {}
        rules = []
        for node in nodes:
            observation = "resources_obs_" + node
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)
            rules.append(ResourcesObserver(ab_pred(node), observation))

        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(resources_info):
        checkInputData.str_data_valid(resources_info, forbidden_chars=[' '])

        return ['resources_obs_' + resources_info]

picosat.SENTENCE_INTERPRETERS[ResourcesObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['resources'] = ResourcesObserver


import unittest


class TestResourcesObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_resources_observer(self):
        with self.assertRaises(ValueError):
            ResourcesObserver(ab_pred(""), "/node")
        with self.assertRaises(ValueError):
            ResourcesObserver(ab_pred("/"), "/node")
        with self.assertRaises(ValueError):
            ResourcesObserver("", "/node")
        with self.assertRaises(ValueError):
            ResourcesObserver("/", "/node")
        with self.assertRaises(ValueError):
            ResourcesObserver(ab_pred("name"), "")
        with self.assertRaises(ValueError):
            ResourcesObserver(ab_pred(""), "")
        with self.assertRaises(ValueError):
            ResourcesObserver("", "")
        with self.assertRaises(TypeError):
            ResourcesObserver(1, "/node")
        with self.assertRaises(TypeError):
            ResourcesObserver(ab_pred("name"), 1)
        with self.assertRaises(ValueError):
            ResourcesObserver(ab_pred("name"), "/")

    def test_clause(self):
        observer = ResourcesObserver(ab_pred("node"), "/node")
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("node"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/node", "Second literal in clause does not match!")

    def test_generate_model_parameter(self):
        config = {'nodes': ['/node1', '/node2'], 'type': 'resources'}
        topics_from_nodes = {}
        vars, rules, nodes = ResourcesObserver.generate_model_parameter(config, topics_from_nodes)

        vars_req = {'resources_obs_/node1': Variable('resources_obs_/node1', 1, None),
                    'resources_obs_/node2': Variable('resources_obs_/node2', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "resources added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [ResourcesObserver(ab_pred('/node1'), 'resources_obs_/node1'),
                     ResourcesObserver(ab_pred('/node2'), 'resources_obs_/node2')]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "resources added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "resources should not add nodes!")

        # test different arguments for the config-parameter which all should raise exeptions
        topics_from_nodes = {}
        with self.assertRaises(KeyError):
            ResourcesObserver.generate_model_parameter({'nodes_wrong_name': ['/topic'], 'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            ResourcesObserver.generate_model_parameter({'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(KeyError):
            ResourcesObserver.generate_model_parameter({}, topics_from_nodes)
        with self.assertRaises(TypeError):
            ResourcesObserver.generate_model_parameter("not_a_dict", topics_from_nodes)
        with self.assertRaises(TypeError):
            ResourcesObserver.generate_model_parameter(1, topics_from_nodes)
        with self.assertRaises(ValueError):
            ResourcesObserver.generate_model_parameter({'nodes': [], 'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(ValueError):
            ResourcesObserver.generate_model_parameter({'nodes': [''], 'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            ResourcesObserver.generate_model_parameter({'nodes': [1], 'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            ResourcesObserver.generate_model_parameter({'nodes': "no_list", 'type': 'resources'}, topics_from_nodes)
        with self.assertRaises(TypeError):
            ResourcesObserver.generate_model_parameter({'nodes': 1, 'type': 'resources'}, topics_from_nodes)

    def test_decrypt_resources_info(self):
        self.assertEqual(ResourcesObserver.decrypt_resource_info("/node1"), ['resources_obs_/node1'], "Topic name decryption not correct!")

        with self.assertRaises(ValueError):
            ResourcesObserver.decrypt_resource_info("/")
        with self.assertRaises(ValueError):
            ResourcesObserver.decrypt_resource_info("")
        with self.assertRaises(ValueError):
            ResourcesObserver.decrypt_resource_info("/node_name ")
        with self.assertRaises(ValueError):
            ResourcesObserver.decrypt_resource_info("/node_name []")
        with self.assertRaises(ValueError):
            ResourcesObserver.decrypt_resource_info("/node_name some_wrong_text")
        with self.assertRaises(TypeError):
            ResourcesObserver.decrypt_resource_info(1)

