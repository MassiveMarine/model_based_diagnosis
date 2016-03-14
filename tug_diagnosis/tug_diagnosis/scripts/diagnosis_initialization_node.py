#!/usr/bin/env python

import rospy

from tug_python_utils import YamlHelper
from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration

from tug_diagnosis_msgs.srv import *

if __name__ == "__main__":
    rospy.init_node('tug_diagnosis_initialization', anonymous=False)
    rospy.loginfo("tug_diagnosis_initialization: starting")
    rospy.loginfo("tug_diagnosis_initialization: load config")
    configs = rospy.get_param('/tug_diagnosis_initialization_node')

    model_configuration = configuration()

    rospy.loginfo("tug_diagnosis_initialization: load nodes")
    for node in YamlHelper.get_param(configs, 'nodes'):
        name = YamlHelper.get_param(node, 'name')
        sub_topic = YamlHelper.get_param(node, 'sub_topic', [])
        pub_topic = YamlHelper.get_param(node, 'pub_topic', [])

        model_configuration.nodes.append(node_configuration(name=str(name),
                                                            pub_topic=list(pub_topic),
                                                            sub_topic=list(sub_topic)))

    rospy.loginfo("tug_diagnosis_initialization: load observers")
    for observation in YamlHelper.get_param(configs, 'observations'):
        type = YamlHelper.get_param(observation, 'type')

        resource = []
        if YamlHelper.has_key(observation, 'nodes'):
            resource = YamlHelper.get_param(observation, 'nodes', [])
        elif YamlHelper.has_key(observation, 'topics'):
            resource = YamlHelper.get_param(observation, 'topics', [])

        model_configuration.observers.append(observer_configuration(type=str(type), resource=list(resource)))

    rospy.loginfo(str(len(model_configuration.nodes)) + " nodes and " +
                  str(len(model_configuration.observers)) + " observers loaded")

    rospy.loginfo("tug_diagnosis_initialization: waiting for service")
    rospy.wait_for_service('diagnosis_configuration_change')
    try:
        diagnosis_changer_srv = rospy.ServiceProxy('diagnosis_configuration_change', DiagnosisConfiguration)
        resp1 = diagnosis_changer_srv(model_configuration, DiagnosisConfigurationRequest.SET)
        error_code = resp1.errorcode
        error_msg = resp1.error_msg
        rospy.loginfo("tug_diagnosis_initialization: configuration sent")
        if error_code is not DiagnosisConfigurationResponse.NO_ERROR:
            raise StandardError(error_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
