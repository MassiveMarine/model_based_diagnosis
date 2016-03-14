from pymbd.sat import picosat
from pymbd.sat.clause import clause
from pymbd.sat.variable import Variable
from pymbd.benchmark.tug_description_parser.observers.base_observer import *


class GeneralObserver(BaseObserver):
    """
    Represents the fault injection logic used to enable/disable a gate's function.
    The implication ab_predicate -> gate_function
    """

    def __init__(self, ab_topic, data):
        super(GeneralObserver, self).__init__()
        checkInputData.str_data_valid(ab_topic)
        checkInputData.dict_data_valid(data, allow_empty=False, check_entries=False)

        for (s, p) in data.values():
            checkInputData.list_data_valid(s, allow_empty=True)
            checkInputData.list_data_valid(p, allow_empty=True)

        self.ab_topic = ab_topic
        self.data = data

    def __repr__(self):
        return "general: %s, %s" % (self.ab_topic, self.data)

    def to_clause(self):
        subs = set()
        [subs.update(s) for (s, p) in self.data.values()]
        clause_list = [clause(neg(self.ab_topic) + " " + all_pos(self.data.keys()) + " " + all_pos(list(subs)))]
        for key, value in self.data.iteritems():
            for ab_topic in value[1]:
                clause_list.append(clause(neg(key) + " " + ab_topic))
        return clause_list

    @staticmethod
    def generate_model_parameter(config, topics_published_from_nodes, topics_subscribed_from_nodes,
                                 nodes_publish_topics, nodes_subscribe_topics):
        topics = config.resource

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

            nodes_publish = topics_published_from_nodes[topic]

            data = {}
            for node in nodes_publish:
                subscribed_topics = nodes_subscribe_topics.get(node, [])
                published_topics = nodes_publish_topics.get(node, [])
                data[ab_pred(str(node))] = (all_ab_pred(subscribed_topics), all_ab_pred(published_topics))

            rules.append(GeneralObserver(ab_pred(topic), data))

        return vars, rules, nodes, []


picosat.SENTENCE_INTERPRETERS[GeneralObserver] = lambda engine, pred, unused: pred.to_clause()
OBSERVERS['general'] = GeneralObserver
