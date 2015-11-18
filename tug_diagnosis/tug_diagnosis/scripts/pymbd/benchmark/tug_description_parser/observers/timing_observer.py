from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class TimingObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function. 
    The implication ab_predicate -> gate_function  
    """
    
    def __init__(self, ab_nodes, topic):
        super(TimingObserver, self).__init__()
        self.ab_nodes = ab_nodes
        self.topic = topic

    def __repr__(self):
        return "(!%s => %s)" % (self.ab_nodes, self.topic)

    def to_clause(self):
        return [clause(all_pos(self.ab_nodes) + " " + self.topic)]

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes):
        topics = config['topics']

        vars = {}
        rules = []
        nodes = []

        for topic_pair in topics:
            topicA = topic_pair[0]
            topicB = topic_pair[1]

            nodes_list = []
            for node in topics_published_from_nodes[topicB]:
                if node in topics_subscribed_from_nodes.get(topicA, []):
                    nodes_list += topics_published_from_nodes[topicB]
                    break

            for node in topics_published_from_nodes[topicA]:
                if node in topics_subscribed_from_nodes.get(topicB, []):
                    nodes_list += topics_published_from_nodes[topicA]

            if not nodes_list:
                nodes_list = topics_published_from_nodes[topicA] +topics_published_from_nodes[topicB]

            for calleridA in topics_published_from_nodes[topicA]:
                for calleridB in topics_published_from_nodes[topicB]:

                    observation = "timing_obs_" + topicA + "_" + calleridA + "_" + topicB + "_" + calleridB
                    vars[observation] = Variable(observation, Variable.BOOLEAN, None)

                    rules.append(TimingObserver(all_ab_pred(nodes_list), observation))

            new_vars, new_rules, new_nodes = CalleridsObserver.generate_model_parameter_2("timing", topicA, topics_published_from_nodes[topicA], topicB, topics_published_from_nodes[topicB] )
            vars.update(new_vars)
            rules += new_rules
            nodes += new_nodes

        return vars, rules, nodes

    @staticmethod
    def decrypt_resource_info(resource_info):

        if not resource_info:
            raise ValueError
        if not isinstance(resource_info, str):
            raise TypeError

        [topicA_name, calleridsA_str, topicB_name, calleridsB_str] = resource_info.split(' ')

        checkInputData.str_data_valid(topicA_name)
        checkInputData.str_data_valid(topicB_name)

        infos = []
        if len(calleridsA_str) <= 2:
            calleridsA = ["all"]
        else:
            calleridsA = [x.strip() for x in calleridsA_str[1:-1].split(',')]

        if len(calleridsB_str) <= 2:
            calleridsB = ["all"]
        else:
            calleridsB = [x.strip() for x in calleridsB_str[1:-1].split(',')]

        for calleridA in calleridsA:
            checkInputData.str_data_valid(calleridA)
            for calleridB in calleridsB:
                checkInputData.str_data_valid(calleridB)
                infos.append('timing_obs_' + str(topicA_name) + "_" + str(calleridA) + "_" + str(topicB_name) + "_" + str(calleridB))

        return infos


picosat.SENTENCE_INTERPRETERS[TimingObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['timing'] = TimingObserver