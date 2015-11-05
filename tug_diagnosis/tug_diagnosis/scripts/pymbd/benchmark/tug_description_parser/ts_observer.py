# from gate import Gate
# from pymbd.util.sethelper import powerset
from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.description import Sentence
from pymbd.sat.variable import Variable

class TsObserver(Sentence):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node, topic):
        self.ab_node = ab_node
        self.topic = topic
        
    def __repr__(self):
        return "(!%s => %s)" % (self.ab_node, self.topic)

    def to_clause(self):
        return [clause(self.ab_node + " " + self.topic)]


def ab_pred(var):
    return 'AB' + var


def generate_model_parameter(obs, topics_from_nodes):
    vars = {}
    rules = []
    for topic in obs['topics']:
        observation = "ts_obs_" + topic
        # vars.append(Variable(observation, Variable.BOOLEAN, 1))
        vars[observation] = Variable(observation, Variable.BOOLEAN, None)
        rules.append(TsObserver(ab_pred(topics_from_nodes[topic]), observation))

    return vars, rules, []


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
    return " !" + literal


def pos(literal):
    return " " + literal


picosat.SENTENCE_INTERPRETERS[TsObserver] = lambda engine, pred, unused: pred.to_clause()
