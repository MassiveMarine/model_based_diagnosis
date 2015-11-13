from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.sat.description import Sentence
from pymbd.benchmark.tug_description_parser.observer import OBSERVERS


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
    def generate_model_parameter(config, topics_from_nodes):
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
    return "!" + literal


def pos(literal):
    return " " + literal


def ab_pred(var):
    return 'AB' + var


class CalleridsObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, group_of_nodes, nodes):
        super(CalleridsObserver, self).__init__()
        # if not ab_node or ab_node is ab_pred("") or not isinstance(ab_node, str):
        #     raise ValueError

        # if not topic or topic is "/" or not isinstance(topic, str):
        #     raise ValueError

        self.group_of_nodes = group_of_nodes
        self.nodes = nodes

    def __repr__(self):
        return "(%s => %s) && (!%s => !%s)" % (self.group_of_nodes, self.nodes, self.group_of_nodes, self.nodes)

    def to_clause(self):

        the_list = []

        for node in self.nodes:
            the_list.append(clause(neg(self.group_of_nodes) + " " + node))

        the_list.append(clause(self.group_of_nodes + " " + all_neg(self.nodes)))

        return the_list


    @staticmethod
    def generate_model_parameter(obs_type, topic, observations):
        vars = {}
        rules = []

        observation_all = obs_type + "_obs_" + topic + "_all"
        observations_w_prefix = [obs_type + "_obs_" + topic + "_" + obs for obs in observations]
        vars[observation_all] = Variable(observation_all, Variable.BOOLEAN, None)
        rules.append(CalleridsObserver(observation_all, observations_w_prefix))
        return vars, rules, []

    @staticmethod
    def decrypt_resource_info(obs_type, topic,):
        return [obs_type + '_obs_' + topic + "_all"]

picosat.SENTENCE_INTERPRETERS[CalleridsObserver] = lambda engine, pred, unused: pred.to_clause()