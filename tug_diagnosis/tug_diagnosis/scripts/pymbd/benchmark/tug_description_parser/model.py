from ab_constraint import AbConstraint
from observer import generate_model_parameter
from observers.base_observer import ab_pred
from sentences import PushSentence
from pymbd.sat.description import Description
from pymbd.sat.problem import Problem
from pymbd.sat.variable import Variable
from config_validator import DiagnosisConfigValidator

from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration
from tug_diagnosis_msgs.srv import DiagnosisConfigurationResponse
import unittest


class ConfigurationValidation(object):
    def __init__(self, **options):
        pass

    @staticmethod
    def compare_list_content_and_len(list1, list2):
        if not set(list1) == set(list2):
            return False
        if not len(list1) == len(list2):
            return False
        return True

    @staticmethod
    def compare_configs(config_1, config_2):
        # test if both configs exist
        if not config_1 or not config_2:
            return False

        # test if both configs have the same nodes
        name_list1, name_list2 = [_.name for _ in config_1.nodes], [_.name for _ in config_2.nodes]
        if not ConfigurationValidation.compare_list_content_and_len(name_list1, name_list2):
            return False

        # compare nodes
        for node1 in config_1.nodes:

            # get corresponding node
            index_if_exists = name_list2.index(node1.name)
            node2 = config_2.nodes[index_if_exists]

            # compare node content
            if not ConfigurationValidation.compare_list_content_and_len(node1.sub_topic, node2.sub_topic):
                return False
            if not ConfigurationValidation.compare_list_content_and_len(node1.pub_topic, node2.pub_topic):
                return False

        # test if both configs have the same observers
        type_list1, type_list2 = [_.type for _ in config_1.observers], [_.type for _ in config_2.observers]
        if not ConfigurationValidation.compare_list_content_and_len(type_list1, type_list2):
            return False

        # compare observers
        for observer1 in config_1.observers:
            match_found = False

            for observer2 in config_2.observers:
                # compare observer content
                if not ConfigurationValidation.compare_list_content_and_len(observer1.resource, observer2.resource):
                    continue
                match_found = True
                break

            if not match_found:
                return False

        return True


class ModelGenerator(object):
    def __init__(self, **options):
        self.config = None

    def test_config(self):
        DiagnosisConfigValidator.minimize_config(self.config)
        # check = DiagnosisConfigValidator(config=self.config, debug=DiagnosisConfigValidator.DEBUG_LEVEL_VERBOSE)
        check = DiagnosisConfigValidator(config=self.config, debug=DiagnosisConfigValidator.DEBUG_LEVEL_DISABLED)
        result = check.run_tests()

        return result

    def set_config(self, set_config):
        config_backup = self.copy(self.config)

        self.config = self.copy(set_config)

        if not self.test_config():
            self.config = config_backup
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='New configuration is not valid!')

        return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.NO_ERROR, error_msg='')

    def add_config(self, config_add):
        """
        Add new nodes and/or observers to model config. If node.name exists, only the pub- and/or sub-topics are added.
        If observer.type exists, only the resource is added.
        :param config_add:
        :return:
        """
        if not self.config:
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='Config need to be set first!')

        if not config_add:
            raise ValueError('No config given for adding')

        config_add = self.copy(config_add)
        config_backup = self.copy(self.config)

        # add to nodes
        for node in config_add.nodes:
            name_list = [_.name for _ in self.config.nodes]
            if node.name in name_list:
                index_if_exists = name_list.index(node.name)
                known_node = self.config.nodes[index_if_exists]
                known_node.sub_topic = list(set(known_node.sub_topic + node.sub_topic))
                known_node.pub_topic = list(set(known_node.pub_topic + node.pub_topic))
            else:
                self.config.nodes.append(node)

        # add to observers
        for observer in config_add.observers:
            match_found = False

            for known_observer in self.config.observers:
                # compare observer content
                if not observer.type == known_observer.type:
                    continue
                if not ConfigurationValidation.compare_list_content_and_len(observer.resource, known_observer.resource):
                    continue
                match_found = True

            if match_found:
                continue

            self.config.observers.append(observer)
        if not self.test_config():
            self.config = config_backup
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='New configuration is not valid!')

        return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.NO_ERROR, error_msg='')

    def remove_config(self, config_remove):
        """
        This removes the configuration for given nodes and/or observers.
        If only nodes.name is given, without pub- or sub-topics, the whole node will be removed.
        If also pub- and/or sub-topics are given, only these topics will be removed. Remaining
        pub- and sub-topics of the node are not removed. The node will also be removed, if it
        has no remaining pub- and sub-topics.

        If only observers.type is given, without resource, all observer of this type will be removed.
        If also resource is given, only these observer will be removed. Other observers of same
        type but with other resource are not removed.
        The observer will also be removed, if it has no remaining resources.
        :param config_remove: config about nodes and observers that should be removed from the model config
        """
        if not self.config:
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='Config need to be set first!')

        if not config_remove:
            raise ValueError('No config given for removing')

        config_remove = self.copy(config_remove)
        config_backup = self.copy(self.config)

        for node in config_remove.nodes:
            # remove of nodes
            name_list = [_.name for _ in self.config.nodes]
            if node.name in name_list:
                index_if_exists = name_list.index(node.name)
                known_node = self.config.nodes[index_if_exists]
                for topic in node.sub_topic:
                    if topic in known_node.sub_topic:
                        known_node.sub_topic.remove(topic)

                for topic in node.pub_topic:
                    if topic in known_node.pub_topic:
                        known_node.pub_topic.remove(topic)

                if not len(node.sub_topic) and not len(node.pub_topic):
                    del self.config.nodes[index_if_exists]

                elif not len(known_node.sub_topic) and not len(known_node.pub_topic):
                    del self.config.nodes[index_if_exists]

        # remove of observers
        for observer in config_remove.observers:
            # find in known observers
            for index, known_observer in enumerate(self.config.observers):
                # continue if name does not fit
                if not observer.type == known_observer.type:
                    continue

                # delete and continue if no resource is given
                if not len(observer.resource):
                    self.config.observers[index] = None
                    continue

                # continue if resource content does not fit
                if not ConfigurationValidation.compare_list_content_and_len(observer.resource, known_observer.resource):
                    continue

                # delete observer because you came so far
                self.config.observers[index] = None
                break

            # deleting from list while iterating is a bad idea, so list is reduced here
            self.config.observers = [x for x in self.config.observers if x]

        if not self.test_config():
            self.config = config_backup
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='New configuration is not valid!')

        return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.NO_ERROR, error_msg='')

    def update_config(self, config_update):
        """
        This is similar to set, but only for given nodes and/or observers. All observers of given type are
        removed and new observers are added.
        :param config_update: new config for given nodes and/or observers
        """
        if not self.config:
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='Config need to be set first!')

        if not config_update:
            raise ValueError('No config given for updating')

        config_update = self.copy(config_update)
        config_backup = self.copy(self.config)

        for node in config_update.nodes:
            # update of nodes
            name_list = [_.name for _ in self.config.nodes]
            if node.name in name_list:
                index_if_exists = name_list.index(node.name)
                del self.config.nodes[index_if_exists]
            self.config.nodes.append(node)

        # remove of observers
        for observer in config_update.observers:
            # find in known observers
            for index, known_observer in enumerate(self.config.observers):
                # continue if name does not fit
                if not observer.type == known_observer.type:
                    continue

                self.config.observers[index] = None

            # deleting from list while iterating is a bad idea, so list is reduced here
            self.config.observers = [x for x in self.config.observers if x]

        for observer in config_update.observers:
            self.config.observers.append(observer)

        if not self.test_config():
            self.config = config_backup
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.CONFIG_INVALID,
                                                  error_msg='New configuration is not valid!')

        return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.NO_ERROR, error_msg='')

    @staticmethod
    def copy(config):
        """
        Make a deep copy of configuration
        :return: deep copy of configuration
        """
        if not config:
            return None

        config_copy = configuration()
        for node in config.nodes:
            config_copy.nodes.append(
                node_configuration(name=str(node.name), pub_topic=list(node.pub_topic), sub_topic=list(node.sub_topic)))

        for observer in config.observers:
            config_copy.observers.append(
                observer_configuration(type=str(observer.type), resource=list(observer.resource)))

        return config_copy

    def get_config_copy(self):
        """
        Get a deep copy of configuration
        :return: deep copy of configuration
        """
        return self.copy(self.config)


class Model(object):
    def __init__(self, **options):
        self.sat_engine_name = options.get('sat_solver', None)
        self.check_problem = Problem(self.sat_engine_name)
        self.comp_problem = self.check_problem
        self.check_queries = 0
        self.comp_queries = 0
        self.queries = 0
        self.options = options
        options['separate_tp'] = options.get('separate_tp', False)
        self.first_check_call = True
        self.first_comp_call = True
        self.previous_diagnoses = set()
        self.last_max_card = 0

        self.vars = {}
        self.rules = []
        self.nodes = []
        self.real_nodes = []

    def set_model(self, configs):
        self.vars = {}
        self.rules = []
        self.nodes = []
        self.real_nodes = []

        topics_published_from_nodes = dict()
        topics_subscribed_from_nodes = dict()
        nodes_publish_topics = dict()
        nodes_subscribe_topics = dict()
        topics = set()
        for node in configs.nodes:
            node_name = node.name
            self.real_nodes.append(node_name)
            self.vars[node_name] = Variable(node_name, Variable.BOOLEAN, None)
            self.vars[ab_pred(node_name)] = Variable(ab_pred(node_name), Variable.BOOLEAN, None)

            for topic in node.pub_topic:
                topics_published_from_nodes.setdefault(topic, []).append(node_name)
                nodes_publish_topics.setdefault(node_name, []).append(topic)

            for topic in node.sub_topic:
                topics_subscribed_from_nodes.setdefault(topic, []).append(node_name)
                nodes_subscribe_topics.setdefault(node_name, []).append(topic)

            topics.update(node.pub_topic)
            topics.update(node.sub_topic)

        topics = list(topics)

        configs.observers.append(observer_configuration(type="general", resource=topics))

        for config in configs.observers:
            new_vars, new_rules, new_nodes, new_real_nodes = generate_model_parameter(config,
                                                                                      topics_published_from_nodes,
                                                                                      topics_subscribed_from_nodes,
                                                                                      nodes_publish_topics,
                                                                                      nodes_subscribe_topics)
            self.vars.update(new_vars)
            self.rules += new_rules
            self.nodes += new_nodes
            self.real_nodes += new_real_nodes

        self.nodes = self.real_nodes + self.nodes

    def set_observations(self, observations):
        """
        Write current observation into diagnosis model.
        :param observations: Is a Tuple (name, value) where 'name' identifies the variable name
                             and 'value' describes the current situation for of the variable.
        """
        for name, value in observations:
            if name in self.vars.keys():
                self.vars[name].value = value
            else:
                print 'A observation is unknown! It will be ignored by the model'

    def set_options(self, **options):
        self.options.update(options)

    def is_real_node(self, node_name):
        """
        Test if given node name is a real node or a generated node.
        :param node_name: node name that should be tested
        :return: True, if it is a real node name otherwise False
        """
        if node_name in self.real_nodes:
            return True
        return False

    def check_consistency(self, h):
        """
        Calculate a conflict set by constraining the AB predicates depending
        on a gates inclusion in h. These new sentences are added to the problem
        and the SAT solver is started again. If it returns SAT, the hitting set
        h is consistent, otherwise it returns UNSAT.
        """
        if self.options['separate_tp']:
            self.check_queries += 1
            # if self.check_queries > 100:
            self.check_queries = 0
            self.check_problem.finished()
            self.check_problem = Problem(self.sat_engine_name)
            self.first_check_call = True
        else:
            self.queries += 1
            # if self.queries > 100:
            self.queries = 0
            self.check_problem.finished()
            self.check_problem = Problem(self.sat_engine_name)
            self.comp_problem = self.check_problem
            self.first_check_call = True
            self.first_comp_call = True

        if self.options['separate_tp'] and self.check_problem == self.comp_problem:
            self.check_problem = Problem(self.sat_engine_name)

        vars = self.vars.values()
        rules = self.rules[:]
        nodes = self.nodes[:]

        # for all gates not in h set the AB predicate to false.
        for node in set(nodes) - h:
            rules.append(AbConstraint(node, False))

        # get me an unsatisfiable core of AB predicates
        r = self.comp_problem.solve(Description(vars, rules), calculate_unsat_core=False)

        return r.sat()

    def calculate_conflicts(self, h):
        """
        Calculate a conflict set by constraining the AB predicates depending
        on a gates inclusion in h. These new sentences are added to the problem
        and the SAT solver is started again. This should return a new UNSAT
        core, which is returned as new conflict set.
        """
        if self.options['separate_tp']:
            self.comp_queries += 1
            # if self.comp_queries > 100:
            self.comp_queries = 0
            self.comp_problem.finished()
            self.comp_problem = Problem(self.sat_engine_name)
            self.first_comp_call = True
        else:
            self.queries += 1
            # if self.queries > 100:
            self.queries = 0
            self.check_problem.finished()
            self.check_problem = Problem(self.sat_engine_name)
            self.comp_problem = self.check_problem
            self.first_check_call = True
            self.first_comp_call = True

        vars = self.vars.values()
        rules = self.rules[:]
        nodes = self.nodes[:]

        rules.append(PushSentence())

        # for all gates not in h set the AB predicate to false.
        for node in set(nodes) - h:
            rules.append(AbConstraint(node, False))

        # get me an unsatisfiable core of AB predicates
        r = self.comp_problem.solve(Description(vars, rules), calculate_unsat_core=True)

        if r.sat():
            return None
        else:
            conflict = map(lambda x: x, r.get_unsat_core())
            return frozenset(conflict)

    def finished(self):
        if self.check_problem:
            self.check_problem.finished()
        if self.comp_problem and self.check_problem != self.comp_problem:
            self.comp_problem.finished()


class TestConfigurationValidation(unittest.TestCase):
    def setUp(self):
        pass

    def test_compare_configs_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertTrue(ConfigurationValidation.compare_configs(config_a, config_b), "configs do not match!")

    def test_compare_configs_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_3(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_4(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1a"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_5(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1d"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_6(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic3"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_7(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic3"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_8(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hza", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_9(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hza", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_10(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1s"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_11(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1s"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_12(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1", "/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_13(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1", "/topic2"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_14(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz1", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_15(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz2", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_16(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        self.assertTrue(ConfigurationValidation.compare_configs(config_a, config_b), "configs do not match!")

    def test_compare_configs_17(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_18(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_19(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_20(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_21(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hzd", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")

    def test_compare_configs_22(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hzw", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        self.assertFalse(ConfigurationValidation.compare_configs(config_a, config_b), "configs should not match!")


class TestModelGenerator(unittest.TestCase):
    def setUp(self):
        pass

    def test_set_config_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_3(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_4(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_5(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_6(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="timestamp", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timestamp", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_7(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_8(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node4", pub_topic=["/topic2"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node4", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_9(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="activated", resource=["node1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node3", pub_topic=["/topic2"], sub_topic=[]))
        config_b.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="activated", resource=["node1"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_10(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_11(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result_a = configuration()
        config_result_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=[]))
        config_result_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_result_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result_a), "configs do not match!")

        config_c = configuration()
        config_c.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))

        config_result_b = configuration()
        config_result_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_result_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic1"], sub_topic=[]))
        config_result_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        gen.add_config(config_c)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result_b), "configs do not match!")

    def test_add_config_12(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()
        # config_result.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_13(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic2"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="timestamp", resource=["/topic2"]))

        config_result = configuration()
        # config_result.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1"]))
        # config_result.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic2"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timestamp", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_14(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_b.observers.append(observer_configuration(type="activated", resource=["node2"]))

        config_result = configuration()
        # config_result.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic2"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="activated", resource=["node2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_15(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=["/topic2"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic2"], sub_topic=["/topic2"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.add_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_add_config_16(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="movement", resource=["/topic1", "/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="movement", resource=["/topic1", "/topic3"]))
        config_b.observers.append(observer_configuration(type="movement", resource=["/topic2", "/topic3"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="movement", resource=["/topic1", "/topic2"]))
        config_result.observers.append(observer_configuration(type="movement", resource=["/topic1", "/topic3"]))
        config_result.observers.append(observer_configuration(type="movement", resource=["/topic2", "/topic3"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        print gen.config
        gen.add_config(config_b)
        print gen.config
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_result = configuration()

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic1"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_3(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", ], sub_topic=[]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node3", pub_topic=[], sub_topic=["/topic1"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic1", ], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic2"], sub_topic=["/topic1"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic3"], sub_topic=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_4(self):
        config_a = configuration()
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=[]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)

        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_5(self):
        config_a = configuration()
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1"))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)

        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_6(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node1", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=[], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_7(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_b.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_8(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=[]))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(
            node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_9(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="hz"))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)

        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_remove_config_10(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="hz", resource=[]))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(
            node_configuration(name="node2", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node3", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="timeout", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.remove_config(config_b)

        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_update_config_1(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=[]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.update_config(config_b)

        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_update_config_2(self):
        config_a = configuration()
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_b = configuration()
        config_b.nodes.append(node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=[]))

        config_result = configuration()
        config_result.nodes.append(node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=[]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.update_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_update_config_3(self):
        config_a = configuration()
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_b = configuration()
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_b.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        gen.update_config(config_b)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.config, config_result), "configs do not match!")

    def test_get_config_copy_1(self):
        config_a = configuration()
        config_a.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_a.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_a.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        config_result = configuration()
        config_result.nodes.append(
            node_configuration(name="node1", pub_topic=["/topic3", "/topic4"], sub_topic=["/topic1", "/topic2"]))
        config_result.nodes.append(node_configuration(name="node2", pub_topic=["/topic1", "/topic2"], sub_topic=[]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic1"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic2"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic3"]))
        config_result.observers.append(observer_configuration(type="hz", resource=["/topic4"]))

        gen = ModelGenerator()
        gen.set_config(config_a)
        self.assertTrue(ConfigurationValidation.compare_configs(gen.get_config_copy(), config_result),
                        "configs do not match!")
