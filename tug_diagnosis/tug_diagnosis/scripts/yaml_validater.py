#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node('tug_yaml_validater', anonymous=False)

    configs = rospy.get_param('/tug_diagnosis_node')

    nodes = []
    topics_published_from_nodes = {}
    topics_subscribed_from_nodes = {}
    nodes_publish_topics = {}
    nodes_subscribe_topics = {}

    topics_from_nodes = set()

    for node in configs['nodes']:
        node_name = node['name']
        nodes.append(node_name)

        for topic in node.get('pub_topic', []):
            topics_published_from_nodes.setdefault(topic, []).append(node_name)
            nodes_publish_topics.setdefault(node_name, []).append(topic)

        for topic in node.get('sub_topic', []):
            topics_subscribed_from_nodes.setdefault(topic, []).append(node_name)
            nodes_subscribe_topics.setdefault(node_name, []).append(topic)

        topics_from_nodes.update(node.get('pub_topic', []))
        topics_from_nodes.update(node.get('sub_topic', []))

    published_topics = [topic for topic in topics_from_nodes if topic in topics_published_from_nodes.keys()]
    not_published_topics = [topic for topic in topics_from_nodes if topic not in topics_published_from_nodes.keys()]
    print 'Topics that are published / not published:   %3d / %3d' % (len(published_topics), len(not_published_topics))
    print not_published_topics

    subscribed_topics = [topic for topic in topics_from_nodes if topic in topics_subscribed_from_nodes.keys()]
    not_subscribed_topics = [topic for topic in topics_from_nodes if topic not in topics_subscribed_from_nodes.keys()]
    print 'Topics that are subscribed / not subscribed: %3d / %3d' % (len(subscribed_topics), len(not_subscribed_topics))
    # print not_subscribed_topics

    topics = set()
    for obs in configs['observations']:
        if 'topics' in obs:
            entry = obs['topics']

            for item in entry:
                if isinstance(item, str):
                    topics.update([item])
                else:
                    topics.update(item)
    print 'Topics that are observerd by diagnosis config: %3d' %(len(topics))

    print 'Topics that are known: %3d' %(len(topics_from_nodes | topics))

    ghost_topics = [topic for topic in topics if topic not in topics_from_nodes]
    print 'Topics observed but no node subscribes or publishs: %3d' % (len(ghost_topics))
    print ghost_topics

    useless_topics = [topic for topic in topics_from_nodes if topic not in topics]
    print 'Topics subscribes or publishs by nodes but not observed: %3d' % (len(useless_topics))
    print useless_topics

    all_topics = topics_from_nodes | topics
    forbidden_topics = set()
    for entry in all_topics:
        if any(x in entry for x in ['$', '#', '|', ' ', '-', 'AB', '/AB']):
            forbidden_topics.update([entry])
    print 'Topics that are with forbidden char: %3d' %(len(forbidden_topics))
    print list(forbidden_topics)







