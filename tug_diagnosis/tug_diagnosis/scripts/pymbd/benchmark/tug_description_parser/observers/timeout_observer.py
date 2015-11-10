from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from base_observer import *


class TimeoutObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, topic):
        BaseObserver.__init__(self)
        self.ab_node = ab_node
        self.topic = topic
        
    def __repr__(self):
        return "(!%s => !%s)" % (self.ab_node, self.topic)

    def to_clause(self):
        return [clause(self.ab_node + " " + neg(self.topic))]

    @staticmethod
    def generate_model_parameter(obs, topics_from_nodes):
        vars = {}
        rules = []
        for topic in obs['topics']:
            observation = "timeout_obs_" + topic
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)
            rules.append(TimeoutObserver(ab_pred(topics_from_nodes[topic]), observation))

        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(resource_info):
        return 'timeout_obs_' + resource_info.split(' ')[0]

picosat.SENTENCE_INTERPRETERS[TimeoutObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timeout'] = TimeoutObserver