from ab_constraint import AbConstraint
# from gate import Gate
# from gate_with_ab_predicate import GateWithAbPredicate, GateSentence
from hz_observer import generate_model_parameter as hz_model_generator
from ts_observer import generate_model_parameter as ts_model_generator
from movement_observer import generate_model_parameter as movement_model_generator
from sentences import PushSentence, PopSentence, BlockingSentence
# from collections import OrderedDict
from pymbd.sat.description import Description
from pymbd.sat.problem import Problem
from pymbd.sat.variable import Variable
# import random


def ab_pred(var):
    return 'AB' + var
        
# def prefix_gate(gate):
#     output = prefix(gate.output)
#     inputs = []
#     for input in gate.inputs:
#         inputs.append(prefix(input))
#     return Gate(output, gate.type, inputs)


class Model(object):
    
    def __init__(self, **options):
        # self.inputs = OrderedDict((i, None) for i in inputs)
        # self.outputs = OrderedDict((o, None) for o in outputs)
        # self.gates = OrderedDict((g.output, g) for g in gates)
        self.sat_engine_name = options.get('sat_solver', None)
        self.check_problem = Problem(self.sat_engine_name)
        self.comp_problem = self.check_problem
        self.check_queries = 0
        self.comp_queries = 0
        self.queries = 0
        self.options = options
        options['reuse_sat'] = options.get('reuse_sat', False)
        options['separate_tp'] = options.get('separate_tp', False)
        self.first_check_call = True
        self.first_comp_call = True
        self.previous_diagnoses = set()
        self.last_max_card = 0

        vars = []
        rules = []

######## movement of imu, loam, odom, cmd
        # vars: alle-topics X alle-observer
#         vars.append(Variable("imu", Variable.BOOLEAN, None))
#         vars.append(Variable("loam", Variable.BOOLEAN, None))
#         vars.append(Variable("base", Variable.BOOLEAN, None))
#         # ABvars: alle Knoten( node(z.B. imu) )
#         vars.append(Variable(ab_pred("imu"), Variable.BOOLEAN, None))
#         vars.append(Variable(ab_pred("loam"), Variable.BOOLEAN, None))
#         vars.append(Variable(ab_pred("base"), Variable.BOOLEAN, None))
#
#
#         vars.append(Variable("hz_obs_imu_topic", Variable.BOOLEAN, 1))
#         vars.append(Variable("hz_obs_loam_topic", Variable.BOOLEAN, 1))
#         vars.append(Variable("hz_obs_odom_topic", Variable.BOOLEAN, 1))
#         vars.append(Variable("hz_obs_cmd_topic", Variable.BOOLEAN, 1))
#
#         vars.append(Variable("movement", Variable.BOOLEAN, None))
#         vars.append(Variable("movement_imu_loam", Variable.BOOLEAN, 0))
#         vars.append(Variable("movement_imu_odom", Variable.BOOLEAN, 0))
#         vars.append(Variable("movement_imu_cmd", Variable.BOOLEAN, 0))
#         vars.append(Variable("movement_loam_odom", Variable.BOOLEAN, 1))
#         vars.append(Variable("movement_loam_cmd", Variable.BOOLEAN, 1))
#         vars.append(Variable("movement_odom_cmd", Variable.BOOLEAN, 1))
#
#
#
#         vars.append(Variable(ab_pred("movement"), Variable.BOOLEAN, None))
#
#         # rules: ein rules-type pro observer type
#         rules.append(HzObserver(ab_pred("imu"), "hz_obs_imu_topic"))
#         rules.append(HzObserver(ab_pred("loam"), "hz_obs_loam_topic"))
#         rules.append(HzObserver(ab_pred("base"), "hz_obs_odom_topic"))
#         rules.append(HzObserver(ab_pred("base"), "hz_obs_cmd_topic"))
#
#         # rules.append(HzObserver(ab_pred("movement"), "imu"))
#         # rules.append(HzObserver(ab_pred("movement"), "loam"))
#         # rules.append(HzObserver(ab_pred("movement"), "odom"))
#
#         rules.append(MovementObserver(ab_pred("imu"), ab_pred("loam"), ab_pred("movement"), "movement_imu_loam"))
#         rules.append(MovementObserver(ab_pred("imu"), ab_pred("base"), ab_pred("movement"), "movement_imu_odom"))
#         rules.append(MovementObserver(ab_pred("imu"), ab_pred("base"), ab_pred("movement"), "movement_imu_cmd"))
#         rules.append(MovementObserver(ab_pred("loam"), ab_pred("base"), ab_pred("movement"), "movement_loam_odom"))
#         rules.append(MovementObserver(ab_pred("loam"), ab_pred("base"), ab_pred("movement"), "movement_loam_cmd"))
#         rules.append(MovementObserver(ab_pred("base"), ab_pred("base"), ab_pred("movement"), "movement_odom_cmd"))

##### imu with hz and ts

        config = {'nodes': [{'name': 'imu', 'pub_topic': ['imu_topic'], 'sub_topic': []},
                            {'name': 'loam', 'pub_topic': ['loam_topic'], 'sub_topic': []},
                            {'name': 'base', 'pub_topic': ['odom_topic', 'cmd_topic'], 'sub_topic': []}],
                  'observations': [{'topics': ['imu_topic',
                              'loam_topic',
                              'odom_topic',
                              'cmd_topic'],
                   'type': 'hz'},
                  {'topics': ['imu_topic',
                              'loam_topic',
                              'odom_topic',
                              'cmd_topic'],
                   'type': 'ts'},
                  {'topics': [['imu_topic', 'loam_topic'],
                              ['imu_topic', 'odom_topic'],
                              ['imu_topic', 'cmd_topic'],
                              ['loam_topic', 'odom_topic'],
                              ['loam_topic', 'cmd_topic'],
                              ['odom_topic', 'cmd_topic']],
                   'type': 'movement'}]}
        vars = {}
        rules = []
        topics_from_nodes = dict()
        nodes = []

        for node in config['nodes']:
            node_name = node['name']
            nodes.append(node_name)
            vars[node_name] = Variable(node_name, Variable.BOOLEAN, None)
            vars[node_name] = Variable(ab_pred(node_name), Variable.BOOLEAN, None)

            for topic in node['pub_topic']:
                topics_from_nodes[topic] = node_name

        for obs in config['observations']:
            new_vars, new_rules, new_nodes = self.get_model_element(obs, topics_from_nodes)
            vars.update(new_vars)
            rules += new_rules
            nodes += new_nodes

        self.temp_vars = vars
        self.temp_rules = rules
        self.temp_nodes = nodes

        self.set_observations([('movement_obs_imu_topic_cmd_topic', 0),
                               ('movement_obs_imu_topic_loam_topic', 0),
                               ('movement_obs_imu_topic_odom_topic', 0),
                               ('movement_obs_loam_topic_odom_topic', 1),
                               ('movement_obs_loam_topic_cmd_topic', 1),
                               ('movement_obs_odom_topic_cmd_topic', 1),
                               ('hz_obs_imu_topic', 1),
                               ('hz_obs_loam_topic', 1),
                               ('hz_obs_odom_topic', 1),
                               ('hz_obs_cmd_topic', 1),
                               ('ts_obs_imu_topic', 1),
                               ('ts_obs_loam_topic', 1),
                               ('ts_obs_odom_topic', 1),
                               ('ts_obs_cmd_topic', 1),
                               ])
    @staticmethod
    def get_model_element(obs, topics_from_nodes):
        if obs['type'] == 'hz':
            print 'HzObserver'
            return hz_model_generator(obs, topics_from_nodes)
        elif obs['type'] == 'ts':
            print 'TsObserver'
            return ts_model_generator(obs, topics_from_nodes)
        elif obs['type'] == 'movement':
            print 'MovementObserver'
            return movement_model_generator(obs, topics_from_nodes)

        return {}, [], []

    def set_observations(self, observations):
        # pass
        for name,value in observations:
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
    

