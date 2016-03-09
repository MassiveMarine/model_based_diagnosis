from pymbd.sat import picosat
from pymbd.sat.clause import Clause
from pymbd.sat.description import Sentence
from pymbd.sat.literal import Literal


class PushSentence(Sentence):
    pass


class PopSentence(Sentence):
    pass


class BlockingSentence(Sentence):
    def __init__(self, solution):
        self.solution = solution


picosat.SENTENCE_INTERPRETERS[PushSentence] = lambda engine, pred, unused: []

picosat.SENTENCE_INTERPRETERS[PopSentence] = lambda engine, pred, unused: []


def blocking_assertion(sentence):
    if len(sentence.solution) > 0:
        r = "(assert (or"
        for s in sentence.solution:
            r += (" (= ABx%s false)" % s)
        r += "))"
        return r
    else:
        return ""


def blocking_clause_cnf(sentence):
    #    print sentence.solution
    return [Clause([Literal('ABx' + x, False) for x in sentence.solution])]
