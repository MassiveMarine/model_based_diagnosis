from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class ScoreObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """
    def __init__(self, ab_node, observation, ab_subscribed_topics):
        super(ScoreObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(observation)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)

        self.ab_node = ab_node
        self.observation = observation
        self.ab_subscribed_topics = ab_subscribed_topics

    def __repr__(self):
        return "score: %s, %s, %s" % (self.ab_node, self.observation, self.ab_subscribed_topics)

    def to_clause(self):
        return [clause(all_pos([self.ab_node] + self.ab_subscribed_topics) + " " + self.observation)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        checkInputData.dict_data_valid(config, False)
        topics = config['topics']

        checkInputData.list_data_valid(topics)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        for topic in topics:
            callerids = topics_published_from_nodes.get(topic, [])
            checkInputData.list_data_valid(callerids)

            for callerid in callerids:
                observation = "score_obs_" + topic + "_" + callerid
                vars[observation] = Variable(observation, Variable.BOOLEAN, None)

                subscribed_topics = nodes_subscribe_topics.get(callerid, [])
                rules.append(ScoreObserver(ab_pred(str(callerid)), observation, all_ab_pred(subscribed_topics)))

                if not set(subscribed_topics).issubset(topics_published_from_nodes.keys()):
                    raise ValueError

            new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter("score", topic, callerids)
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
            return ['score_obs_' + str(topic_name) + "_all"]

        callerids = [x.strip() for x in callerids_str[1:-1].split(',')]

        infos = []
        for callerid in callerids:
            checkInputData.str_data_valid(callerid)
            infos.append('score_obs_' + str(topic_name) + "_" + str(callerid))

        return infos


picosat.SENTENCE_INTERPRETERS[ScoreObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['score'] = ScoreObserver


import unittest


class TestScoreObserver(unittest.TestCase):
    def setUp(self):
        pass

    def test_score_observer1(self):
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
                ScoreObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_score_observer2(self):
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
                ScoreObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_score_observer3(self):
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
                ScoreObserver(ab_node, observation, ab_subscribed_topics)
            print "... DONE"

    def test_clause(self):
        observer = ScoreObserver(ab_pred("name"), "/topic", [])
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 2, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), "/topic", "Second literal in clause does not match!")

        observer = ScoreObserver(ab_pred("name"), "/topic", all_ab_pred(['/topic1']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 3, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), "/topic", "Third literal in clause does not match!")

        observer = ScoreObserver(ab_pred("name"), "/topic", all_ab_pred(['/topic1', '/topic2']))
        new_clause = observer.to_clause()[0]
        self.assertEqual(len(new_clause.literals), 4, "Number of 'literals' in clause does not match!")
        self.assertEqual(str(new_clause.literals[0]), ab_pred("name"), "First literal in clause does not match! ")
        self.assertEqual(str(new_clause.literals[1]), ab_pred("/topic1"), "Second literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[2]), ab_pred("/topic2"), "Third literal in clause does not match!")
        self.assertEqual(str(new_clause.literals[3]), "/topic", "Fourth literal in clause does not match!")

    def test_generate_model_parameter1(self):
        config = {'topics': ['/topic'], 'type': 'score'}
        topics_published_from_nodes = {'/topic': ['/node1', '/node2']}
        topics_subscribed_from_nodes = {}
        nodes_publish_topics = {'/node1': ['/topic'], '/node2': ['/topic']}
        nodes_subscribe_topics = {}

        vars, rules, nodes = ScoreObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)

        vars_req = {'score_obs_/topic_all': Variable('score_obs_/topic_all', 1, None),
                    'score_obs_/topic_/node1': Variable('score_obs_/topic_/node1', 1, None),
                    'score_obs_/topic_/node2': Variable('score_obs_/topic_/node2', 1, None)}

        self.assertEqual(len(vars), len(vars_req), "score added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        subscribed_topics = []
        rules_req = [ScoreObserver(ab_pred('/node1'), 'score_obs_/topic_/node1', subscribed_topics),
                     ScoreObserver(ab_pred('/node2'), 'score_obs_/topic_/node2', subscribed_topics),
                     CalleridsObserver('score_obs_/topic_all', ['score_obs_/topic_/node1', 'score_obs_/topic_/node2'])]

        rules_req_str = [str(x) for x in rules_req]
        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "score added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "score should not add nodes!")

    def test_generate_model_parameter2(self):
        config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'score'}
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}
        vars, rules, nodes = ScoreObserver.generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)

        vars_req = {'score_obs_/topic1_all': Variable('score_obs_/topic1_all', 1, None),
                    'score_obs_/topic1_node1': Variable('score_obs_/topic1_node1', 1, None),
                    'score_obs_/topic2_all': Variable('score_obs_/topic2_all', 1, None),
                    'score_obs_/topic2_node2': Variable('score_obs_/topic2_node2', 1, None),
                    'score_obs_/topic3_all': Variable('score_obs_/topic3_all', 1, None),
                    'score_obs_/topic3_node3': Variable('score_obs_/topic3_node3', 1, None),
                    }

        self.assertEqual(len(vars), len(vars_req), "score added wrong number of variables!")
        for i, obj in vars.items():
            self.assertTrue(vars_req.has_key(i), "Key '" + str(i) + "' not in variables-required list!")
            self.assertEqual(str(vars_req[i]), str(obj), "Variable '" + str(i) + "' not generated with right parameters!")

        rules_req = [ScoreObserver(ab_pred('node1'), 'score_obs_/topic1_node1', all_ab_pred([])),
                     CalleridsObserver('score_obs_/topic1_all', ['score_obs_/topic1_node1']),
                     ScoreObserver(ab_pred('node2'), 'score_obs_/topic2_node2', all_ab_pred(['/topic1'])),
                     CalleridsObserver('score_obs_/topic2_all', ['score_obs_/topic2_node2']),
                     ScoreObserver(ab_pred('node3'), 'score_obs_/topic3_node3', all_ab_pred(['/topic2'])),
                     CalleridsObserver('score_obs_/topic3_all', ['score_obs_/topic3_node3']),]

        rules_req_str = [str(x) for x in rules_req]

        self.assertTrue(not any([x for x in rules if str(x) not in rules_req_str]), "Rules does not match!")
        self.assertEqual(len(rules), len(rules_req), "score added wrong number of rules!")
        self.assertEqual(len(nodes), 0, "score should not add nodes!")

    def test_generate_model_parameter_errors_1(self):
        # test different arguments for the config-parameter which all should raise exeptions
        config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'score'}
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        config_tests = [(KeyError, {'topics_wrong_name': ['/topic'], 'type': 'score'}),
                        (KeyError, {'type': 'score'}),
                        (KeyError, {}),
                        (TypeError, "not_a_dict"),
                        (TypeError, 1),
                        (ValueError, {'topics': [], 'type': 'score'}),
                        (ValueError, {'topics': [''], 'type': 'score'}),
                        (TypeError, {'topics': [1], 'type': 'score'}),
                        (TypeError, {'topics': "no_list", 'type': 'score'}),
                        (TypeError, {'topics': 1, 'type': 'score'}),
                        (ValueError, {'topics': ['/topic', '/topic2', '/topic3'], 'type': 'score'})
                        ]

        for (error, config) in config_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(config) + "'",

                ScoreObserver.generate_model_parameter(config,
                                                   topics_published_from_nodes, topics_subscribed_from_nodes,
                                                   nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_2(self):
        config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'score'}
        topics_published_from_nodes = {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}
        topics_subscribed_from_nodes = {'node3': ['/topic2'], 'node2': ['/topic1']}
        nodes_publish_topics = {'node1': ['/topic1'], 'node3': ['/topic3'], 'node2': ['/topic2']}
        nodes_subscribe_topics = {'node3': ['/topic2'], 'node2': ['/topic1']}

        topics_published_from_nodes_testes = \
            [(ValueError, {'/topic':  ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': []       , '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic':  ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': []       , '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic':  ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': []       }),
             (ValueError, {1:  ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/':  ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'':  ['node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (KeyError, {}),
             (TypeError, "no_dict"),
             (TypeError, 1),
             (ValueError, {'/topic3': ['/' ,'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['' ,'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (TypeError, {'/topic3': [1 ,'node3'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3', '/'], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3', ''], '/topic2': ['node2'], '/topic1': ['node1']}),
             (TypeError, {'/topic3': ['node3', 1], '/topic2': ['node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['/', 'node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['', 'node2'], '/topic1': ['node1']}),
             (TypeError, {'/topic3': ['node3'], '/topic2': [1, 'node2'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2', '/'], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2', ''], '/topic1': ['node1']}),
             (TypeError, {'/topic3': ['node3'], '/topic2': ['node2', 1], '/topic1': ['node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['/', 'node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['', 'node1']}),
             (TypeError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': [1, 'node1']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', '/']}),
             (ValueError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', '']}),
             (TypeError, {'/topic3': ['node3'], '/topic2': ['node2'], '/topic1': ['node1', 1]}),
             ]

        for (error, topics_published_from_nodes) in topics_published_from_nodes_testes:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(topics_published_from_nodes) + "'",

                ScoreObserver.generate_model_parameter(config,
                                                   topics_published_from_nodes, topics_subscribed_from_nodes,
                                                   nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_generate_model_parameter_errors_3(self):
        config = {'topics': ['/topic1', '/topic2', '/topic3'], 'type': 'score'}
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

                ScoreObserver.generate_model_parameter(config,
                                                   topics_published_from_nodes, topics_subscribed_from_nodes,
                                                   nodes_publish_topics, nodes_subscribe_topics)
            print "... DONE"

    def test_decrypt_resource_info(self):
        self.assertEqual(ScoreObserver.decrypt_resource_info("/topic_name [node1, node2]"), ['score_obs_/topic_name_node1', 'score_obs_/topic_name_node2'], "Topic name decryption not correct!")
        self.assertEqual(ScoreObserver.decrypt_resource_info("/topic_name [node1, node2, node3, node4, node5]"), ['score_obs_/topic_name_node1', 'score_obs_/topic_name_node2', 'score_obs_/topic_name_node3',
                                                                                                               'score_obs_/topic_name_node4', 'score_obs_/topic_name_node5'], "Topic name decryption not correct!")
        self.assertEqual(ScoreObserver.decrypt_resource_info("/topic_name [node1]"), ['score_obs_/topic_name_node1'], "Topic name decryption not correct!")
        self.assertEqual(ScoreObserver.decrypt_resource_info("/topic_name {}"), ['score_obs_/topic_name_all'], "Topic name decryption not correct!")

        resource_info_tests = [
            (ValueError, "/ [node1, node2]"),
            (ValueError, "/ [node1, /]"),
            (ValueError, "/topic_name"),
            (ValueError, "/topic_name [/]"),
            (ValueError, "/topic_name [/node1, ]"),
            (ValueError, "/topic_name [/node1, /]"),
            (ValueError, "/"),
            (ValueError, ""),
            (TypeError, 1),
            ]

        for (error, resource_info) in resource_info_tests:
            with self.assertRaises(error):
                print "'" + str(error.__name__) + "' should be raised by '" + str(resource_info) + "'",
                ScoreObserver.decrypt_resource_info(resource_info)
            print "... DONE"
