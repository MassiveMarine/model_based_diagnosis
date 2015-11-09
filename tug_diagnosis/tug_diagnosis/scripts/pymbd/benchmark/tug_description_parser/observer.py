
OBSERVERS = {}


def generate_model_parameter(obs, topics_from_nodes):
    return OBSERVERS[obs['type']].generate_model_parameter(obs, topics_from_nodes)


def decrypt_resource_info(obs):
    return OBSERVERS[obs[0]].decrypt_resource_info(obs[1])

from observers import *
