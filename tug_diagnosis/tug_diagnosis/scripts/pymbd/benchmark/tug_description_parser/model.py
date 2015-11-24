from ab_constraint import AbConstraint
from observer import generate_model_parameter
from observers.base_observer import ab_pred
from sentences import PushSentence
from pymbd.sat.description import Description
from pymbd.sat.problem import Problem
from pymbd.sat.variable import Variable


class Model(object):
    
    def __init__(self, configs, **options):
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

        self.configs = configs

        vars = {}
        rules = []
        topics_published_from_nodes = dict()
        topics_subscribed_from_nodes = dict()
        nodes_publish_topics = dict()
        nodes_subscribe_topics = dict()
        nodes = []
        topics = set()

        for node in configs['nodes']:
            node_name = node['name']
            nodes.append(node_name)
            vars[node_name] = Variable(node_name, Variable.BOOLEAN, None)
            vars[ab_pred(node_name)] = Variable(ab_pred(node_name), Variable.BOOLEAN, None)

            for topic in node.get('pub_topic', []):
                topics_published_from_nodes.setdefault(topic, []).append(node_name)
                nodes_publish_topics.setdefault(node_name, []).append(topic)

            for topic in node.get('sub_topic', []):
                topics_subscribed_from_nodes.setdefault(topic, []).append(node_name)
                nodes_subscribe_topics.setdefault(node_name, []).append(topic)

            topics.update(node.get('pub_topic', []))
            topics.update(node.get('sub_topic', []))

        topics = list(topics)
        config_for_general = {'topics': topics, 'type': 'general'}

        configs['observations'].append(config_for_general)

        for config in configs['observations']:
            new_vars, new_rules, new_nodes = generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics)
            vars.update(new_vars)
            rules += new_rules
            nodes += new_nodes

        self.temp_vars = vars
        self.temp_rules = rules
        self.temp_nodes = nodes

    def set_observations(self, observations):
        # pass
        for name, value in observations:
            self.temp_vars[name].value = value

    def set_options(self, **options):
        self.options.update(options)

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

        vars = self.temp_vars.values()
        rules = self.temp_rules[:]
        nodes = self.temp_nodes[:]

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

        vars = self.temp_vars.values()
        rules = self.temp_rules[:]
        nodes = self.temp_nodes[:]

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
    

