from pymbd.sat import picosat
from pymbd.sat.clause import Clause
from pymbd.sat.description import Sentence
from pymbd.sat.literal import Literal


class AbConstraint(Sentence):
    def __init__(self, ab_predicate, value, extended=True, weight=None):
        self.ab_predicate = ab_predicate
        self.value = value
        self.extended = extended
        self.weight = weight


def ab_constraint(engine, ab_constraint):
    # yices spits out "id: x" lines for each assert+ we give
    # This records each AbConstraint to map the ids back to names later on
    if ab_constraint.extended:
        engine.register_core_sentence(None, ab_constraint.ab_predicate)
        weight_str = " %d" % ab_constraint.weight if ab_constraint.weight else ""
        if ab_constraint.value:
            return "(assert+ (= AB%s true)%s)" % (ab_constraint.ab_predicate, weight_str)
        else:
            return "(assert+ (= AB%s false)%s)" % (ab_constraint.ab_predicate, weight_str)
    else:
        if ab_constraint.value:
            return "(assert (= AB%s true))" % ab_constraint.ab_predicate
        else:
            return "(assert (= AB%s false))" % ab_constraint.ab_predicate


def clause_ab_constraint(engine, ab_constraint):
    """
    >>> clause_ab_constraint(AbConstraint('x10', True))
    [[ABx10]]
    """
    engine.register_core_sentence("AB" + ab_constraint.ab_predicate, ab_constraint)
    return [Clause([Literal("AB" + ab_constraint.ab_predicate, ab_constraint.value)], weight=ab_constraint.weight)]


picosat.SENTENCE_INTERPRETERS[AbConstraint] = lambda engine, p, unused: clause_ab_constraint(engine, p)

if __name__ == "__main__":
    import doctest

    doctest.testmod()
