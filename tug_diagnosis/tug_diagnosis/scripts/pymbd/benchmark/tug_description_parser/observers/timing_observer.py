from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from base_observer import *


class TimingObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, topics):
        BaseObserver.__init__(self)
        self.ab_node = ab_node
        self.topics = topics

    def __repr__(self):
        return "(!%s => %s)" % (self.ab_node, self.topics)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.topics)]

    @staticmethod
    def generate_model_parameter(obs, topics_from_nodes):
        vars = {}
        rules = []
        nodes = []

        for topic in obs['topics']:
            observation = "timing_obs_" + topic[0] + "_" + topic[1]
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)
            rules.append(TimingObserver(ab_pred(topics_from_nodes[topic[0]]), observation))

        return vars, rules, nodes

    @staticmethod
    def decrypt_resource_info(resource_info):
        infos = resource_info.split(' ')
        return 'timing_obs_' + infos[0] + '_' + infos[2]


picosat.SENTENCE_INTERPRETERS[TimingObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timing'] = TimingObserver