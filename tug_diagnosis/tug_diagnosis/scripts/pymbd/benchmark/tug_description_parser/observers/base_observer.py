from pymbd.sat.description import Sentence
from ..observer import OBSERVERS


class BaseObserver(Sentence):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    def __init__(self):
        pass

    def __repr__(self):
        raise NotImplementedError()

    def to_clause(self):
        raise NotImplementedError()

    @staticmethod
    def generate_model_parameter(obs, topics_from_nodes):
        raise NotImplementedError("generate_model_parameter() is not implemented yet!")

    @staticmethod
    def decrypt_resource_info(resource_info):
        raise NotImplementedError("decrypt_resource_info() is not implemented yet!")


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


def ab_pred(var):
    return 'AB' + var