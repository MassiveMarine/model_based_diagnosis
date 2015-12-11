#!/usr/bin/env python
from pymbd.diagnosis.problem import Problem
from pymbd.benchmark.tug_description_parser.oracle import TUGDescriptionOracle
from pymbd.util.sethelper import write_sets

from pymbd.benchmark.tug_description_parser.observers import *
from pymbd.benchmark.tug_description_parser.observer import OBSERVERS

import rospy
from tug_observers_msgs.msg import observer_info
from tug_diagnosis_msgs.msg import diagnosis_set, diagnosis, resource_mode_assignement
from std_msgs.msg import Header
from observation_store import ObservationStore
import threading


class Diagnosis(object):

    def __init__(self):
        self._observation_store = ObservationStore()
        self._trigger_condition = threading.Condition()
        self._observer_sub = rospy.Subscriber("/observers/info", observer_info, self.observer_callback)
        self._diagnosis_pub = rospy.Publisher('/diagnosis', diagnosis_set, queue_size=10)

    def observer_callback(self, observations):
        for obs in observations.observation_infos:
            # print obs
            self._observation_store.add_observation(obs.type, obs.resource, obs.observation, obs.header.stamp)

        if self._observation_store.has_changed():
            self._trigger_condition.acquire()
            self._trigger_condition.notify_all()
            self._trigger_condition.release()

    def run(self):

        p = Problem()
        the_list = ['hst-picosat',
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

        o = TUGDescriptionOracle(configs, ())

        while not rospy.is_shutdown():
            self._trigger_condition.acquire()
            self._trigger_condition.wait()

            observations = self._observation_store.get_observations()

            if all( [j for (i,j) in observations]):
                continue

            self._trigger_condition.release()

            o.observations = observations
            r = p.compute_with_description(o, the_list[6])
            d = r.get_diagnoses()
            d = map(o.numbers_to_nodes, d)

            corrupt_nodes = []

            msg = diagnosis_set()
            msg.header = Header(stamp=rospy.Time.now())
            msg.type = the_list[6]

            for diag in d:

                diagnosis_entry = diagnosis()
                corrupt_set = []
                for node in diag:

                    if not o.is_real_node(node):
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

            rospy.loginfo( "new diagnosis done in " + str(r.get_stats()['total_time']) + " with '" + str(the_list[6]) + "':")
            for corrupt_node in corrupt_nodes:
                rospy.loginfo(str(corrupt_node))
            if not len(corrupt_nodes):
                rospy.loginfo('no solution')


if __name__ == "__main__":
    rospy.init_node('tug_diagnosis', anonymous=False)

    configs = rospy.get_param('/tug_diagnosis_node')

    the_diagnostics = Diagnosis()

    the_diagnostics.run()
