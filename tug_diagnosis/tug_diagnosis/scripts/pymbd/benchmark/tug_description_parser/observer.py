OBSERVERS = {}


def generate_model_parameter(config,
                             topics_published_from_nodes,
                             topics_subscribed_from_nodes,
                             nodes_publish_topics,
                             nodes_subscribe_topics):
    # return OBSERVERS[config['type']].generate_model_parameter(config,
    #                                                           topics_published_from_nodes,
    #                                                           topics_subscribed_from_nodes,
    #                                                           nodes_publish_topics,
    #                                                           nodes_subscribe_topics)
    return OBSERVERS[config.type].generate_model_parameter(config,
                                                           topics_published_from_nodes,
                                                           topics_subscribed_from_nodes,
                                                           nodes_publish_topics,
                                                           nodes_subscribe_topics)


def decrypt_resource_info(obs):
    return OBSERVERS[obs[0]].decrypt_resource_info(obs[1])
