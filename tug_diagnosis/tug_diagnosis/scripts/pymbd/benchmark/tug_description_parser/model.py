from ab_constraint import AbConstraint
from observer import generate_model_parameter
from observers.base_observer import ab_pred
from sentences import PushSentence
from pymbd.sat.description import Description
from pymbd.sat.problem import Problem
from pymbd.sat.variable import Variable

from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration


class ModelGenerator(object):
    def __init__(self):
        self.config = None

    def set_config(self, set_config):
        print "set_config"
        self.config = set_config

    def add_config(self, add_config):
        print "add_config"

    def remove_config(self, remove_config):
        print "remove_config"

    def update_config(self, update_config):
        print "update_config"


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
            new_vars, new_rules, new_nodes, new_real_nodes = generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)
            self.vars.update(new_vars)
            self.rules += new_rules
            self.nodes += new_nodes
            self.real_nodes += new_real_nodes

        self.nodes = self.real_nodes + self.nodes

    def set_observations(self, observations):
        # pass
        for name, value in observations:
            if name in self.vars.keys():
                self.vars[name].value = value

    def set_options(self, **options):
        self.options.update(options)

    def is_real_node(self, node_name):
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
        if self.options['separate_tp'] == True:
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

        if self.options['separate_tp'] == True and self.check_problem == self.comp_problem:
            self.check_problem = Problem(self.sat_engine_name)

        vars = self.vars.values()
        rules = self.rules[:]
        nodes = self.nodes[:]

        # for all gates not in h set the AB predicate to false.
        for node in set(nodes)-h:
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
        if self.options['separate_tp'] == True:
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
        for node in set(nodes)-h:
            rules.append(AbConstraint(node, False))

        # get me an unsatisfiable core of AB predicates
        r = self.comp_problem.solve(Description(vars, rules), calculate_unsat_core=True)

        if (r.sat()):
            return None
        else:
            conflict = map(lambda x: x, r.get_unsat_core())
            return frozenset(conflict)

    def finished(self):
        if self.check_problem:
            self.check_problem.finished()
        if self.comp_problem and self.check_problem != self.comp_problem:
            self.comp_problem.finished()
    

