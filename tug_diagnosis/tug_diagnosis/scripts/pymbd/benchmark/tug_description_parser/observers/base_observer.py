from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.sat.description import Sentence
from pymbd.benchmark.tug_description_parser.observer import OBSERVERS
import unittest


class BaseObserver(Sentence):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self):
        pass

    def __repr__(self):
        raise NotImplementedError()

    def to_clause(self):
        raise NotImplementedError()

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        raise NotImplementedError("generate_model_parameter() is not implemented yet!")

    @staticmethod
    def decrypt_resource_info(resource_info):
        raise NotImplementedError("decrypt_resource_info() is not implemented yet!")


def all_neg(literals):
    """
    >>> all_neg(['x1', 'x2', 'x3'])
    '!x1 !x2 !x3'
    >>> all_neg(['x1'])
    '!x1'
    """
    return "!" + " !".join(literals)


def all_pos(literals):
    return " ".join(literals)


def neg(literal):
    return "!" + literal


def pos(literal):
    return " " + literal


def all_ab_pred(var):
    return [ab_pred(var_entry) for var_entry in var]


def ab_pred(var):
    return 'AB' + var


def get_node_depends_on_nodes_list(node_name, topics_published_from_nodes, topics_subscribed_from_nodes,
                                   nodes_publish_topics, nodes_subscribe_topics):
    sub_topics = nodes_subscribe_topics.get(node_name, [])
    checkInputData.list_data_valid(sub_topics, allow_empty=True)
    depend_on_nodes = []
    [depend_on_nodes.extend(topics_published_from_nodes.get(x)) for x in sub_topics]
    checkInputData.list_data_valid(depend_on_nodes, allow_empty=True)
    return depend_on_nodes


class checkInputData():
    @staticmethod
    def str_data_valid(data, forbidden_chars=[], forbidden_str=[]):
        if not data:
            raise ValueError

        if not isinstance(data, str):
            raise TypeError

        forbidden_strings_list = ['/', '', ab_pred("/"), ab_pred("")] + forbidden_str
        if any(x == data for x in forbidden_strings_list):
            raise ValueError("Forbidden strings found! '" + str(forbidden_strings_list) + "' are not allowed. '" + str(
                data) + "' use one of these.")

        forbidden_chars_list = ['$', '#', '|'] + forbidden_chars
        if any(x in data for x in forbidden_chars_list):
            raise ValueError("Forbidden characters found! '" + str(forbidden_chars_list) + "' are not allowed. '" + str(
                data) + "' use one of these.")

    @staticmethod
    def list_data_valid(the_list, check_entries=True, allow_empty=False, num_entries=0):
        if not isinstance(the_list, list):
            raise TypeError
        if not len(the_list) and not allow_empty:
            raise ValueError('List is empty!')
        if num_entries > 0 and not len(the_list) == num_entries:
            raise ValueError('List has wrong number of entries!')

        if check_entries:
            for entry in the_list:
                checkInputData.str_data_valid(entry)

    @staticmethod
    def dict_data_valid(the_dict, check_entries=True, entry_type=str, allow_empty=False):
        if not isinstance(the_dict, dict):
            raise TypeError
        if not len(the_dict) and not allow_empty:
            raise KeyError

        if check_entries:
            for entry in the_dict.values():
                if entry_type == str:
                    checkInputData.str_data_valid(entry)
                if entry_type == list:
                    checkInputData.list_data_valid(entry)


class CalleridsObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, nodes_all, nodes):
        super(CalleridsObserver, self).__init__()
        # if not nodes_all or nodes_all == ab_pred("/") or nodes_all == ab_pred("") or nodes_all == "/" or
        #  nodes_all == "":
        #     raise ValueError
        # if not isinstance(nodes_all, str):
        #     raise TypeError
        checkInputData.str_data_valid(nodes_all)

        # if not isinstance(nodes, list):
        #     raise TypeError
        # if not len(nodes):
        #     raise ValueError
        #
        # for node in nodes:
        #     if not node or node == ab_pred("/") or node == ab_pred("") or node == "/" or node == "":
        #         raise ValueError
        #     if not isinstance(node, str):
        #         raise TypeError
        checkInputData.list_data_valid(nodes)

        self.nodes_all = nodes_all
        self.nodes = nodes

    def __repr__(self):
        return str("%s => %s)" % (self.nodes_all, self.nodes))

    def to_clause(self):

        the_list = []

        for node in self.nodes:
            the_list.append(clause(neg(self.nodes_all) + " " + node))

        the_list.append(clause(self.nodes_all + " " + all_neg(self.nodes)))

        return the_list

    @staticmethod
    def generate_model_parameter(obs_type, topic, callerids):
        vars = {}
        rules = []

        checkInputData.str_data_valid(obs_type)
        checkInputData.str_data_valid(topic)
        checkInputData.list_data_valid(callerids)

        observation_all = obs_type + "_obs_" + topic + "_all"
        observations_w_prefix = [obs_type + "_obs_" + topic + "_" + obs for obs in callerids]
        vars[observation_all] = Variable(observation_all, Variable.BOOLEAN, None)
        rules.append(CalleridsObserver(observation_all, observations_w_prefix))
        return vars, rules, []

    @staticmethod
    def generate_model_parameter_2(obs_type, topic_a, callerids_a, topic_b, callerids_b):
        vars = {}
        rules = []

        checkInputData.str_data_valid(obs_type)
        checkInputData.str_data_valid(topic_a)
        checkInputData.str_data_valid(topic_b)
        checkInputData.list_data_valid(callerids_a)
        checkInputData.list_data_valid(callerids_b)

        observation_all_all = obs_type + "_obs_" + topic_a + "_all_" + topic_b + "_all"
        vars[observation_all_all] = Variable(observation_all_all, Variable.BOOLEAN, None)

        observations_all_items = [obs_type + "_obs_" + topic_a + "_all_" + topic_b + "_" + obs_item for obs_item in
                                  callerids_b]
        rules.append(CalleridsObserver(observation_all_all, observations_all_items))

        observations_items_all = [obs_type + "_obs_" + topic_a + "_" + obs_item + "_" + topic_b + "_all" for obs_item in
                                  callerids_a]
        rules.append(CalleridsObserver(observation_all_all, observations_items_all))

        for node_a in callerids_a:
            observations_item_all = obs_type + "_obs_" + topic_a + "_" + node_a + "_" + topic_b + "_all"
            vars[observations_item_all] = Variable(observations_item_all, Variable.BOOLEAN, None)
            observations_item_items = [obs_type + "_obs_" + topic_a + "_" + node_a + "_" + topic_b + "_" + obs for obs
                                       in callerids_b]
            rules.append(CalleridsObserver(observations_item_all, observations_item_items))

        for node_b in callerids_b:
            observations_all_item = obs_type + "_obs_" + topic_a + "_all_" + topic_b + "_" + node_b
            vars[observations_all_item] = Variable(observations_all_item, Variable.BOOLEAN, None)
            observations_items_item = [obs_type + "_obs_" + topic_a + "_" + obs + "_" + topic_b + "_" + node_b for obs
                                       in callerids_a]
            rules.append(CalleridsObserver(observations_all_item, observations_items_item))

        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(obs_type, topic, ):
        checkInputData.str_data_valid(obs_type)
        checkInputData.str_data_valid(topic)
        return [obs_type + '_obs_' + topic + "_all"]


picosat.SENTENCE_INTERPRETERS[CalleridsObserver] = lambda engine, pred, unused: pred.to_clause()


class TestCalleridsObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_callerids_observer(self):
        with self.assertRaises(ValueError):
            CalleridsObserver("", ["/node1"])
        with self.assertRaises(ValueError):
            CalleridsObserver("/", ["/node1"])
        with self.assertRaises(TypeError):
            CalleridsObserver(1, ["/node1"])
        with self.assertRaises(ValueError):
            CalleridsObserver(ab_pred(""), ["/node1"])
        with self.assertRaises(ValueError):
            CalleridsObserver(ab_pred("/"), ["/node1"])
        with self.assertRaises(TypeError):
            CalleridsObserver(ab_pred(1), ["/node1"])

        with self.assertRaises(ValueError):
            CalleridsObserver("node_all", [])
        with self.assertRaises(ValueError):
            CalleridsObserver("node_all", [""])
        with self.assertRaises(ValueError):
            CalleridsObserver("node_all", ["/"])
        with self.assertRaises(TypeError):
            CalleridsObserver("node_all", [1])
        with self.assertRaises(TypeError):
            CalleridsObserver("node_all", "/")
        with self.assertRaises(TypeError):
            CalleridsObserver("node_all", 1)
        with self.assertRaises(ValueError):
            CalleridsObserver("node_all", ["node1", ""])
        with self.assertRaises(ValueError):
            CalleridsObserver("node_all", ["node1", "/"])
        with self.assertRaises(TypeError):
            CalleridsObserver("node_all", ["node1", 1])

    def test_clause(self):
        observer = CalleridsObserver("name_all", ["node1", "node2"])
        clauses = observer.to_clause()
        self.assertEqual(len(clauses), 3, "Number of 'clauses' does not match!")
        self.assertEqual(len(clauses[0].literals), 2, "Number of 'literals' in first clause does not match!")
        self.assertEqual(len(clauses[1].literals), 2, "Number of 'literals' in second clause does not match!")
        self.assertEqual(len(clauses[2].literals), 3, "Number of 'literals' in third clause does not match!")
        self.assertEqual(str(clauses[0].literals[0]), "!name_all", "First literal in first clause does not match! ")
        self.assertEqual(str(clauses[0].literals[1]), "node1", "Second literal in first clause does not match! ")
        self.assertEqual(str(clauses[1].literals[0]), "!name_all", "First literal in second clause does not match! ")
        self.assertEqual(str(clauses[1].literals[1]), "node2", "Second literal in second clause does not match! ")
        self.assertEqual(str(clauses[2].literals[0]), "name_all", "First literal in third clause does not match! ")
        self.assertEqual(str(clauses[2].literals[1]), "!node1", "Second literal in third clause does not match! ")
        self.assertEqual(str(clauses[2].literals[2]), "!node2", "Third literal in third clause does not match! ")

    def test_generate_model_parameter(self):
        vars, rules, nodes = CalleridsObserver.generate_model_parameter("hz", "/topic1", ["/node1", "node2"])

        vars_req = {'hz_obs_/topic1_all': Variable('hz_obs_/topic1_all', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "Hz added wrong number of variables!")

        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj),
                             "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [CalleridsObserver('hz_obs_/topic1_all', ['hz_obs_/topic1_/node1', 'hz_obs_/topic1_node2'])]

        self.assertEqual(str(rules[0]), str(rules_req[0]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "Added wrong number of rules!")
        self.assertEqual(len(rules), 1, "Callerids observer should add only one rule!")
        self.assertEqual(len(nodes), 0, "Callerids observer should not add nodes!")

        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("", "topic1", ["node1", "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("/", "topic1", ["node1", "node2"])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter(1, "topic1", ["node1", "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "", ["node1", "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "/", ["node1", "node2"])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", 1, ["node1", "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", ["", "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", ["/", "node2"])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [1, "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [ab_pred("/"), "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [ab_pred(""), "node2"])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [ab_pred(1), "node2"])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", ["node1", ""])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", ["node1", "/"])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", ["node1", 1])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [])
        with self.assertRaises(ValueError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", [""])
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", "")
        with self.assertRaises(TypeError):
            CalleridsObserver.generate_model_parameter("hz", "topic1", 1)

    def test_decrypt_resource_info(self):
        self.assertEqual(CalleridsObserver.decrypt_resource_info("hz", "/topic1"), ['hz_obs_/topic1_all'],
                         "Topic name decryption not correct!")

        with self.assertRaises(ValueError):
            CalleridsObserver.decrypt_resource_info("", "/topic1")
        with self.assertRaises(ValueError):
            CalleridsObserver.decrypt_resource_info("/", "/topic1")
        with self.assertRaises(TypeError):
            CalleridsObserver.decrypt_resource_info(1, "/topic1")
        with self.assertRaises(ValueError):
            CalleridsObserver.decrypt_resource_info("hz", "")
        with self.assertRaises(ValueError):
            CalleridsObserver.decrypt_resource_info("hz", "/")
        with self.assertRaises(TypeError):
            CalleridsObserver.decrypt_resource_info("hz", 1)
