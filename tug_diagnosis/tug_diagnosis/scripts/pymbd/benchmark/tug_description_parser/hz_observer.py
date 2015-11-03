# from gate import Gate
# from pymbd.util.sethelper import powerset
from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.description import Sentence

class HzObserver(Sentence):
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


# def clauses_gate_with_ab_predicate(ab, gate):
#     c = CNF[gate.type](gate)
#     return map(lambda i: clause(i + pos(ab)), c)
#
#
# def clauses_gate(gate):
#     c = CNF[gate.type](gate)
#     return map(lambda i: clause(i), c)

# CNF = {
#     # these are the CNFs of the boolean functions used as gates, i.e. for
#     # c <=> a OP b.
#     # obtained using wolframalpha.com (just enter c <=> (a xor b) for example)
#     # Note that nand, nor, and and or have been extended to work with multiple inputs
#
#     # For AND, in logic notation, the CNF would be (LaTeX)
#     # z \Leftrightarrow \bigwedge_i x_i \equiv \left( \bigvee_i (\neg x_i) \vee z \right) \wedge \left(\bigwedge_i (x_i \vee \neg z) \right)
#
#     'nand' : lambda gate: [all_neg(gate.inputs) + neg(gate.output)] + map(lambda input: pos(input) + pos(gate.output), gate.inputs),
#     'nor'  : lambda gate: [all_pos(gate.inputs) + pos(gate.output)] + map(lambda input: neg(input) + neg(gate.output), gate.inputs),
#     'and'  : lambda gate: [all_neg(gate.inputs) + pos(gate.output)] + map(lambda input: pos(input) + neg(gate.output), gate.inputs),
#     'or'   : lambda gate: [all_pos(gate.inputs) + neg(gate.output)] + map(lambda input: neg(input) + pos(gate.output), gate.inputs),
#     'xor'  : lambda gate: map(lambda x: x % {'a': gate.inputs[0], 'b': gate.inputs[1], 'c': gate.output},
#                               ["!%(a)s !%(b)s !%(c)s", "!%(a)s %(b)s %(c)s", "%(a)s !%(b)s %(c)s", "%(a)s %(b)s !%(c)s"]),
#     'xnor' : lambda gate: map(lambda x: x % {'a': gate.inputs[0], 'b': gate.inputs[1], 'c': gate.output},
#                               ["!%(a)s !%(b)s %(c)s", "!%(a)s %(b)s !%(c)s", "%(a)s !%(b)s !%(c)s", "%(a)s %(b)s %(c)s"]),
#     'buff' : lambda gate: map(lambda x: x % {'a': gate.inputs[0], 'c': gate.output},
#                               ["!%(a)s %(c)s", "%(a)s !%(c)s"]),
#     'not'  : lambda gate: map(lambda x: x % {'a': gate.inputs[0], 'c': gate.output},
#                               ["!%(a)s !%(c)s", "%(a)s %(c)s"])
# }

picosat.SENTENCE_INTERPRETERS[HzObserver] = lambda engine, pred, unused: pred.to_clause()
# picosat.SENTENCE_INTERPRETERS[GateSentence]        = lambda engine, pred, unused: clauses_gate(pred.gate)


# def cnf_gate_with_ab_predicate(ab, gate):
#     c = CNF[gate.type](gate)
#     return "\n".join(map(lambda i: "(assert (or " + cnf_clause(i) + " " + ab + "))", c))
#
#
# def cnf_clause(c):
#     return " ".join(map(lambda l: cnf_literal(l), clause(c).literals))
#
#
# def cnf_literal(l):
#     return l.name if l.sign else "(not %s)" % l.name

# if __name__ == "__main__":
#     import doctest
#     doctest.testmod()
#