from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class GeneralObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """
    def __init__(self, ab_node, ab_topic, ab_subscribed_topics, ab_published_topics):
        super(GeneralObserver, self).__init__()
        checkInputData.str_data_valid(ab_node)
        checkInputData.str_data_valid(ab_topic)
        checkInputData.list_data_valid(ab_subscribed_topics, allow_empty=True)
        checkInputData.list_data_valid(ab_published_topics, allow_empty=True)

        self.ab_node = ab_node
        self.ab_topic = ab_topic
        self.ab_subscribed_topics = ab_subscribed_topics
        self.ab_published_topics = ab_published_topics

    def __repr__(self):
        return "general: %s, %s" % (self.ab_node, self.ab_topic)

    def to_clause(self):
        clause_list = [clause(neg(self.ab_topic) + " " + self.ab_node + " " + all_pos(self.ab_subscribed_topics))]
        for ab_topic in self.ab_published_topics:
            clause_list.append(clause(neg(self.ab_node) + " " + ab_topic))
        return clause_list

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes, nodes_publish_topics, nodes_subscribe_topics):
        checkInputData.dict_data_valid(config, False)
        topics = config['topics']

        checkInputData.list_data_valid(topics)

        checkInputData.dict_data_valid(topics_published_from_nodes, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(topics_subscribed_from_nodes, check_entries=False, allow_empty=True)
        checkInputData.dict_data_valid(nodes_publish_topics, check_entries=False, allow_empty=False)
        checkInputData.dict_data_valid(nodes_subscribe_topics, check_entries=False, allow_empty=True)

        vars = {}
        rules = []
        nodes = []

        for topic in topics:
            checkInputData.list_data_valid(topics_published_from_nodes[topic])

            vars[topic] = Variable(topic, Variable.BOOLEAN, None)
            vars[ab_pred(topic)] = Variable(ab_pred(topic), Variable.BOOLEAN, None)
            nodes.append(topic)

            for node in topics_published_from_nodes[topic]:
                subscribed_topics = nodes_subscribe_topics.get(node, [])
                published_topics = nodes_publish_topics.get(node, [])
                rules.append(GeneralObserver(ab_pred(str(node)), ab_pred(topic), all_ab_pred(subscribed_topics), all_ab_pred(published_topics) ))

        return vars, rules, nodes, []


picosat.SENTENCE_INTERPRETERS[GeneralObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['general'] = GeneralObserver

