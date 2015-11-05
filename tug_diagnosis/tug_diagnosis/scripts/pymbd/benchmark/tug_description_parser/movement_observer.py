from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.description import Sentence
from pymbd.sat.variable import Variable

class MovementObserver(Sentence):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_node_a, ab_node_b, ab_function, observation):
        self.ab_node_a = ab_node_a
        self.ab_node_b = ab_node_b
        self.ab_function = ab_function
        self.observation = observation

    def __repr__(self):
        return "(!%s && !%s &*& !%s => %s)" % (self.ab_node_a, self.ab_node_b, self.ab_function, self.observation)

    def to_clause(self):
        return [ clause(self.ab_node_a + " " + self.ab_node_b + " " + self.ab_function + " " + self.observation)]


def ab_pred(var):
    return 'AB' + var


def generate_model_parameter(obs, topics_from_nodes):
    vars = {}
    rules = []
    nodes = []

    nodes.append("movement")
    vars["movement"] = Variable("movement", Variable.BOOLEAN, None)
    vars[ab_pred("movement")] = Variable(ab_pred("movement"), Variable.BOOLEAN, None)

    for topic in obs['topics']:
        observation = "movement_obs_" + topic[0] + "_"  + topic[1]
        vars[observation] = Variable(observation, Variable.BOOLEAN, None)
        rules.append(MovementObserver(ab_pred(topics_from_nodes[topic[0]]), ab_pred(topics_from_nodes[topic[1]]), ab_pred("movement"), observation))

    return vars, rules, nodes


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

picosat.SENTENCE_INTERPRETERS[MovementObserver] = lambda engine, pred, unused: pred.to_clause()
