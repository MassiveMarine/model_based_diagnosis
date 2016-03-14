#!/usr/bin/env python
from pymbd.diagnosis.problem import Problem
from pymbd.benchmark.tug_description_parser.oracle import TUGDescriptionOracle
from pymbd.util.sethelper import write_sets

from pymbd.benchmark.tug_description_parser.observers import *
from pymbd.benchmark.tug_description_parser.observer import OBSERVERS

import rospy
from tug_observers_msgs.msg import observer_info
from tug_diagnosis_msgs.msg import diagnosis_set, diagnosis, resource_mode_assignement
from tug_diagnosis_msgs.srv import *
from std_msgs.msg import Header
from observation_store import ObservationStore
import threading


# from tug_diagnosis_msgs.msg import configuration, node_configuration, observer_configuration

class Diagnosis(object):
    def __init__(self):
        self._observation_store = ObservationStore()
        self._trigger_condition = threading.Condition()

        self.p = Problem()
        self.solver_list = ['hst-picosat',
                            'hst-cache-picosat',
                            'hst-ci-picosat',
                            'hst-ci-cache-picosat',
                            'hsdag-picosat',
                            'hsdag-cache-picosat',
                            'hsdag-ci-picosat',
                            'hsdag-ci-cache-picosat',
                            'hsdag-sicf-picosat',
                            'hsdag-sicf-cache-picosat'
                            ]

        self.o = TUGDescriptionOracle()

        self._observer_sub = rospy.Subscriber("/observers/info", observer_info, self.observer_callback)
        self._diagnosis_pub = rospy.Publisher('/diagnosis', diagnosis_set, queue_size=10)
        self._service = rospy.Service('diagnosis_configuration_change', DiagnosisConfiguration,
                                      self.handle_diagnosis_configuration_change)

    def observer_callback(self, observations):
        for obs in observations.observation_infos:
            # print obs
            self._observation_store.add_observation(obs.type, obs.resource, obs.observation, obs.header.stamp)

        if self._observation_store.has_changed():
            self._trigger_condition.acquire()
            self._trigger_condition.notify_all()
            self._trigger_condition.release()

    def handle_diagnosis_configuration_change(self, req):
        action = req.action
        config = req.config

        if action == DiagnosisConfigurationRequest.ADD:
            result = self.o.net_generator.add_config(config)
        elif action == DiagnosisConfigurationRequest.REMOVE:
            result = self.o.net_generator.remove_config(config)
        elif action == DiagnosisConfigurationRequest.SET:
            result = self.o.net_generator.set_config(config)
        elif action == DiagnosisConfigurationRequest.UPDATE:
            result = self.o.net_generator.update_config(config)
        else:
            return DiagnosisConfigurationResponse(errorcode=DiagnosisConfigurationResponse.GENERAL_ERROR, error_msg='unknown action')

        self.o.setup = False

        print self.o.net_generator.config

        return result

    def run(self):

        while not rospy.is_shutdown():
            self._trigger_condition.acquire()
            self._trigger_condition.wait()
            try:
                observations = self._observation_store.get_observations()
            except ValueError as e:
                rospy.logerr(e)
                continue

            # check if all observations are valid
            if all([j for (i, j) in observations]):
                continue

            self._trigger_condition.release()

            self.o.observations = observations
            r = self.p.compute_with_description(self.o, self.solver_list[6])
            d = r.get_diagnoses()
            d = map(self.o.numbers_to_nodes, d)

            corrupt_nodes = []

            msg = diagnosis_set()
            msg.header = Header(stamp=rospy.Time.now())
            msg.type = self.solver_list[6]

            for diag in d:

                diagnosis_entry = diagnosis()
                corrupt_set = []
                for node in diag:

                    if not self.o.is_real_node(node):
                        continue
                    corrupt_set.append(node)

                    attr = resource_mode_assignement()
                    attr.resource = node
                    attr.mode_msg = "'%s' seems to be corrupt!" % node
                    attr.mode = attr.GENERAL_ERROR
                    diagnosis_entry.diagnosis.append(attr)

                corrupt_nodes.append(corrupt_set)
                msg.diagnoses.append(diagnosis_entry)

            self._diagnosis_pub.publish(msg)

            rospy.loginfo("new diagnosis done in " + str(r.get_stats()['total_time']) + " with '" + str(
                self.solver_list[6]) + "':")
            for corrupt_node in corrupt_nodes:
                rospy.loginfo(str(corrupt_node))
            if not len(corrupt_nodes):
                rospy.loginfo('no solution')


if __name__ == "__main__":
    rospy.init_node('tug_diagnosis', anonymous=False)

    the_diagnostics = Diagnosis()

    the_diagnostics.run()
