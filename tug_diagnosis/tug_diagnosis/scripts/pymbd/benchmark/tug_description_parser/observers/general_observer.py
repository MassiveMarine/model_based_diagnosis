from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class GeneralObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """
    def __init__(self, ab_node, ab_topic, ab_subscribed_topics, ab_published_topics):
        super(GeneralObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(ab_topic)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)
        checkInputData.list_data_valid(ab_published_topics, allow_empty=True)

        self.ab_node = ab_node
        self.ab_topic = ab_topic
        self.ab_subscribed_topics = ab_subscribed_topics
        self.ab_published_topics = ab_published_topics

    def __repr__(self):
        return "general: %s, %s" % (self.ab_node, self.ab_topic)

    def to_clause(self):
        clause_list = [clause(neg(self.ab_topic) + " " + self.ab_node + " " + all_pos(self.ab_subscribed_topics))]
        for ab_topic in self.ab_published_topics:
            clause_list.append(clause(neg(self.ab_node) + " " + ab_topic))
        return clause_list

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics):
        checkInputData.dict_data_valid(config, False)
        topics = config['topics']

        checkInputData.list_data_valid(topics)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(topics_subscribed_from_nodes, check_entries=False, allow_empty=True)
        checkInputData.dict_data_valid(nodes_publish_topics, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        for topic in topics:
            checkInputData.list_data_valid(topics_published_from_nodes[topic])

            vars[topic] = Variable(topic, Variable.BOOLEAN, None)
            vars[ab_pred(topic)] = Variable(ab_pred(topic), Variable.BOOLEAN, None)
            nodes.append(topic)

            for node in topics_published_from_nodes[topic]:
                subscribed_topics = nodes_subscribe_topics.get(node, [])
                published_topics = nodes_publish_topics.get(node, [])
                rules.append(GeneralObserver(ab_pred(str(node)), ab_pred(topic), all_ab_pred(subscribed_topics), all_ab_pred(published_topics) ))

        return vars, rules, nodes


picosat.SENTENCE_INTERPRETERS[GeneralObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['general'] = GeneralObserver


# import unittest
#
#
# class TestGeneralObserver(unittest.TestCase):
#     def setUp(self):
#         pass
#
#     def test_general_observer(self):
#         with self.assertRaises(ValueError):
#             GeneralObserver(ab_pred(""), "/topic")
#         with self.assertRaises(ValueError):
#             GeneralObserver(ab_pred("/"), "/topic")
#         with self.assertRaises(ValueError):
#             GeneralObserver("", "/topic")
#         with self.assertRaises(ValueError):
#             GeneralObserver("/", "/topic")
#         with self.assertRaises(ValueError):
#             GeneralObserver(ab_pred("name"), "")
#         with self.assertRaises(ValueError):
#             GeneralObserver(ab_pred(""), "")
#         with self.assertRaises(ValueError):
#             GeneralObserver("", "")
#         with self.assertRaises(TypeError):
#             GeneralObserver(1, "/topic")
#         with self.assertRaises(TypeError):
#             GeneralObserver(ab_pred("name"), 1)
#         with self.assertRaises(ValueError):
#             GeneralObserver(ab_pred("name"), "/")
#
#     def test_clause(self):
#         observer = GeneralObserver(ab_pred("name"), "/topic")
#         new_clause = observer.to_clause()[0]
#         self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
#         self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
#         self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")
#
#     def test_generate_model_parameter(self):
#         config = {'topics': ['/topic'], 'type': 'general'}
#         topics_from_nodes = {'/topic': ['/node1', '/node2']}
#         vars, rules, nodes = GeneralObserver.generate_model_parameter(config, topics_from_nodes)
#
#         vars_req = {'general_obs_/topic_all': Variable('general_obs_/topic_all', 1, None),
#                     'general_obs_/topic_/node1': Variable('general_obs_/topic_/node1', 1, None),
#                     'general_obs_/topic_/node2': Variable('general_obs_/topic_/node2', 1, None)}
#
#         self.assertEqual(len(vars), len(vars_req), "general added wrong number of variables!")
#         for i, obj in vars.items():
#             self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
#             self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")
#
#         rules_req = [GeneralObserver(ab_pred('/node1'), 'general_obs_/topic_/node1'),
#                      GeneralObserver(ab_pred('/node2'), 'general_obs_/topic_/node2'),
#                      CalleridsObserver('general_obs_/topic_all', ['general_obs_/topic_/node1', 'general_obs_/topic_/node2'])]
#
#         rules_req_str = [str(x) for x in rules_req]
#         self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
#         self.assertEqual(len(rules), len(rules_req), "general added wrong number of rules!")
#         self.assertEqual(len(nodes), 0, "general should not add nodes!")
#
#         # test different arguments for the config-parameter which all should raise exeptions
#         topics_from_nodes = {'/topic': ['/node1', '/node2']}
#         with self.assertRaises(KeyError):
#             GeneralObserver.generate_model_parameter({'topics_wrong_name': ['/topic'], 'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(KeyError):
#             GeneralObserver.generate_model_parameter({'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(KeyError):
#             GeneralObserver.generate_model_parameter({}, topics_from_nodes)
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter("not_a_dict", topics_from_nodes)
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter(1, topics_from_nodes)
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter({'topics': [], 'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter({'topics': [''], 'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter({'topics': [1], 'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter({'topics': "no_list", 'type': 'general'}, topics_from_nodes)
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter({'topics': 1, 'type': 'general'}, topics_from_nodes)
#
#         # test different arguments for the topics_from_nodes-parameter which all should raise exeptions
#         config = {'topics': ['/topic'], 'type': 'general'}
#         with self.assertRaises(KeyError):
#             GeneralObserver.generate_model_parameter(config, {'/topic_wrong_name': ['/node1', '/node2']})
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': []})
#         with self.assertRaises(KeyError):
#             GeneralObserver.generate_model_parameter(config, {})
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter(config, "no_dict")
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter(config, 1)
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': ['/', '/node2']})
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': ['', '/node2']})
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': [1, '/node2']})
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': ['/node1', '/']})
#         with self.assertRaises(ValueError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': ['/node1', '']})
#         with self.assertRaises(TypeError):
#             GeneralObserver.generate_model_parameter(config, {'/topic': ['/node1', 1]})
#
#     def test_decrypt_resource_info(self):
#         self.assertEqual(GeneralObserver.decrypt_resource_info("/topic_name [node1, node2]"), ['general_obs_/topic_name_node1', 'general_obs_/topic_name_node2'], "Topic name decryption not correct!")
#         self.assertEqual(GeneralObserver.decrypt_resource_info("/topic_name [node1, node2, node3, node4, node5]"), ['general_obs_/topic_name_node1', 'general_obs_/topic_name_node2', 'general_obs_/topic_name_node3',
#                                                                                                                'general_obs_/topic_name_node4', 'general_obs_/topic_name_node5'], "Topic name decryption not correct!")
#         self.assertEqual(GeneralObserver.decrypt_resource_info("/topic_name [node1]"), ['general_obs_/topic_name_node1'], "Topic name decryption not correct!")
#         self.assertEqual(GeneralObserver.decrypt_resource_info("/topic_name {}"), ['general_obs_/topic_name_all'], "Topic name decryption not correct!")
#
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/ [node1, node2]")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/ [node1, /]")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/topic_name")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/topic_name [/]")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/topic_name [/node1, ]")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/topic_name [/node1, /]")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("/")
#         with self.assertRaises(ValueError):
#             GeneralObserver.decrypt_resource_info("")
#         with self.assertRaises(TypeError):
#             GeneralObserver.decrypt_resource_info(1)

