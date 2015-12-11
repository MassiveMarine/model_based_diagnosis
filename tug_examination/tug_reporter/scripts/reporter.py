#!/usr/bin/env python
import rospy
import threading
import thread
from tug_observers_msgs.msg import observer_info
from tug_diagnosis_msgs.msg import diagnosis_set
from observation_store import ObservationStore
from diagnosis_store import DiagnosisStore
from tug_python_utils import YamlHelper
from rules import RuleFactory

__author__ = 'clemens'


class Reporter(object):

    def __init__(self):
        self._observer_sub = rospy.Subscriber("/observer/info", observer_info, self.observer_callback)
        self._diagnosis_sub = rospy.Subscriber("/diagnosis", diagnosis_set, self.diagnosis_callback)
        self._observation_store = ObservationStore()
        self._diagnosis_store = DiagnosisStore()
        self._trigger_condition = threading.Condition()

        self._rules = []
        rule_configs = rospy.get_param(rospy.get_name() + "/rules")
        for rule_config in rule_configs:
            rule_type = YamlHelper.get_param(rule_config, "type")
            self._rules.append(RuleFactory.create_rule(rule_type, rule_config))

        for rule in self._rules:
            self._observation_store.observations_used(rule.observations_to_use())
            self._diagnosis_store.resources_used(rule.resources_to_use())

        self._background_thread = thread.start_new_thread(self.run, tuple())

    def observer_callback(self, observations):
        for obs in observations.observation_infos:
            self._observation_store.add_observation(obs.type, obs.resource, obs.observation, obs.header.stamp)

        if self._observation_store.has_changed():
            self._trigger_condition.acquire()
            self._trigger_condition.notify_all()
            self._trigger_condition.release()

    def diagnosis_callback(self, diagnoses):
        self._diagnosis_store.add_diagnosis(diagnoses.type, diagnoses.diagnoses, diagnoses.header.stamp)

        if self._diagnosis_store.has_changed():
            self._trigger_condition.acquire()
            self._trigger_condition.notify_all()
            self._trigger_condition.release()

    def run(self):
        while not rospy.is_shutdown():
            self._trigger_condition.acquire()
            self._trigger_condition.wait()
            self._trigger_condition.release()

            for rule in self._rules:
                if rule.can_trigger(self._observation_store, self._diagnosis_store):
                    rule.trigger()
                pass


if __name__ == '__main__':
    rospy.init_node('tug_reporter')

    the_reporter = Reporter()

    rospy.spin()
