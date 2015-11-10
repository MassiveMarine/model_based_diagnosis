from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from base_observer import *


class ResourceObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, node):
        BaseObserver.__init__(self)
        self.ab_node = ab_node
        self.node = node
        
    def __repr__(self):
        return "(!%s => %s)" % (self.ab_node, self.node)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.node)]

    @staticmethod
    def generate_model_parameter(obs, topics_from_nodes):
        vars = {}
        rules = []
        for node in obs['nodes']:
            observation = "resource_obs_" + node
            vars[observation] = Variable(observation, Variable.BOOLEAN, None)
            rules.append(ResourceObserver(ab_pred(topics_from_nodes[node]), observation))

        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(resource_info):
        return 'resource_obs_' + resource_info.split(' ')[0]

picosat.SENTENCE_INTERPRETERS[ResourceObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['resource'] = ResourceObserver