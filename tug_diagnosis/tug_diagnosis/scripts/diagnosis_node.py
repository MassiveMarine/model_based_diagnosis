#!/usr/bin/env python
from pymbd.diagnosis.problem import Problem
from pymbd.benchmark.tug_description_parser.oracle import TUGDescriptionOracle
from pymbd.util.sethelper import write_sets

from pymbd.benchmark.tug_description_parser.observers import *
from pymbd.benchmark.tug_description_parser.observer import OBSERVERS

import rospy
from tug_observers_msgs.msg import observer_info
from tug_diagnosis_msgs.msg import diagnosis_set
from observation_store import ObservationStore
import threading


class Diagnosis(object):

    def __init__(self):
        self._observation_store = ObservationStore()
        self._trigger_condition = threading.Condition()
        self._observer_sub = rospy.Subscriber("/observers/info", observer_info, self.observer_callback)
        # self._diagnosis_pub = rospy.Publisher("/diagnosis", diagnosis_set)

    def observer_callback(self, observations):
        for obs in observations.observation_infos:
            # print obs
            self._observation_store.add_observation(obs.type, obs.resource, obs.observation, obs.header.stamp)

        if self._observation_store.has_changed():
            self._trigger_condition.acquire()
            self._trigger_condition.notify_all()
            self._trigger_condition.release()

    def run(self):

        # rospy.spin()
        while not rospy.is_shutdown():
            self._trigger_condition.acquire()
            self._trigger_condition.wait()


            rospy.loginfo( "do diagnisis")

            observations = self._observation_store.get_observations()

            self._trigger_condition.release()

            p = Problem()

            the_list = ['hst-picosat']
            # try:
            o = TUGDescriptionOracle(configs, observations)
            r = p.compute_with_description(o, 'hst-picosat')#, max_card=3)
            d = r.get_diagnoses()
            d = map(o.numbers_to_nodes, d)
            rospy.loginfo(str(r.get_stats()['total_time']) + " " + write_sets(d))
            # except AttributeError as e:
            #     rospy.logwarn('Can not be solved')










if __name__ == "__main__":
    rospy.init_node('tug_diagnosis', anonymous=False)

    configs = rospy.get_param('/tug_diagnosis_node')

    the_diagnostics = Diagnosis()

    the_diagnostics.run()


#     p = Problem()
#
#     the_list = ['hst-picosat',
#             # 'hst-cache-picosat',
#             # 'hst-ci-picosat',
#             # 'hst-ci-cache-picosat',
#             # 'hsdag-picosat',
#             # 'hsdag-cache-picosat',
#             # 'hsdag-ci-picosat',
#             # 'hsdag-ci-cache-picosat',
#             # 'hsdag-sicf-picosat',
#             # 'hsdag-sicf-cache-picosat'
#             ]
#
#     configs = {'nodes': [{'name': 'imu', 'pub_topic': ['imu_topic'], 'sub_topic': []},
#                          {'name': 'loam', 'pub_topic': ['loam_topic'], 'sub_topic': []},
#                          {'name': 'base','pub_topic': ['odom_topic', 'cmd_topic'], 'sub_topic': []}],
#                'observations': [{'topics': ['imu_topic', 'loam_topic', 'odom_topic', 'cmd_topic'], 'type': 'hz'},
#                                 {'topics': [['imu_topic', 'loam_topic'],
#                                             ['imu_topic', 'odom_topic'],
#                                             ['imu_topic', 'cmd_topic'],
#                                             ['loam_topic', 'odom_topic'],
#                                             ['loam_topic', 'cmd_topic'],
#                                             ['odom_topic', 'cmd_topic']], 'type': 'movement'}
#                                 ]
#                }
#
# # for i in the_list:
#     observations = [
#                     ('movement_obs_imu_topic_cmd_topic', 0),
#                     ('movement_obs_imu_topic_loam_topic', 0),
#                     ('movement_obs_imu_topic_odom_topic', 0),
#                     ('movement_obs_loam_topic_odom_topic', 1),
#                     ('movement_obs_loam_topic_cmd_topic', 1),
#                     ('movement_obs_odom_topic_cmd_topic', 1),
#                     ('hz_obs_imu_topic', 1),
#                     ('hz_obs_loam_topic', 1),
#                     ('hz_obs_odom_topic', 1),
#                     ('hz_obs_cmd_topic', 1),
#                     # ('ts_obs_imu_topic', 1),
#                     # ('ts_obs_loam_topic', 1),
#                     # ('ts_obs_odom_topic', 1),
#                     # ('ts_obs_cmd_topic', 1),
#                     ]
#
#     try:
#         o = TUGDescriptionOracle(configs, observations)
#         r = p.compute_with_description(o, 'hst-picosat', max_card=3)
#         d = r.get_diagnoses()
#         d = map(o.numbers_to_nodes, d)
#         print str(r.get_stats()['total_time']) + " " + write_sets(d)
#     except AttributeError as e:
#         print 'Can not be solved'
# #
