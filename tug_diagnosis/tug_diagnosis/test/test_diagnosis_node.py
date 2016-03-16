#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from tug_diagnosis_msgs.msg import diagnosis_set, diagnosis, resource_mode_assignement
from tug_observers_msgs.msg import observer_info, observation_info, observation

from threading import Lock, Condition

import unittest

import rostest

global pub_tester
pub_tester = None
global observer_pub
observer_pub = None

observation_ok = observation(observation_msg='ok', verbose_observation_msg='everything ok', observation=1)
observation_error = observation(observation_msg='fail', verbose_observation_msg='something failed', observation=-1)


def resource_mode_assignment_ok(name):
    return resource_mode_assignement(resource=name, mode_msg="'" + str(name) + "' seems to be corrupt!", mode=1)


def resource_mode_assignment_error(name):
    return resource_mode_assignement(resource=name, mode_msg="'" + str(name) + "' seems to be corrupt!", mode=-1)


class PublisherTester:
    def __init__(self, subscription_topic_name, subscription_class):
        self._the_sub = rospy.Subscriber(subscription_topic_name, subscription_class, self.sub_cb)
        self._the_mutex = Lock()
        self._should_use_subscriber_content = False
        self._buffered_content = None
        self._got_message_condition = Condition(self._the_mutex)

    def sub_cb(self, cb_msg):
        rospy.logdebug("got message")
        self._got_message_condition.acquire()

        if self._should_use_subscriber_content:
            rospy.logdebug("buffer message")
            self._buffered_content = cb_msg
            self._got_message_condition.notify_all()

        self._got_message_condition.release()
        pass

    def get_message(self, function_to_call, time_to_wait):
        self._the_mutex.acquire()
        self._should_use_subscriber_content = True

        rospy.logdebug("call function")
        function_to_call()
        rospy.logdebug("function called")

        self._got_message_condition.wait(time_to_wait)
        result = self._buffered_content
        print result

        self._should_use_subscriber_content = False
        self._the_mutex.release()
        return result


def check_list_in_list(compare_fct, first, second):
    # find each element of the first list in the second list
    for entry_of_first in first:
        first_found_in_second = False
        for entry_of_second in second:
            if compare_fct(entry_of_first, entry_of_second):
                first_found_in_second = True
                break
        if not first_found_in_second:
            return False
    return True


def compare_resource_mode_assignement(first, second):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # it's not allowed, if resources, mode_msg, or mode are different
    if not first.resource == second.resource:
        return False
    if not first.mode_msg == second.mode_msg:
        return False
    if not first.mode == second.mode:
        return False
    return True


def compare_diagnosis(first, second):
    # find each element of the first list in the second list and each element of the second list in the first list
    if not check_list_in_list(compare_resource_mode_assignement, first.diagnosis, second.diagnosis):
        return False
    if not check_list_in_list(compare_resource_mode_assignement, second.diagnosis, first.diagnosis):
        return False
    return True


def compare_diagnosis_set(first, second):
    # if both are equal, they can also be None
    if not first and not second:
        return True

    # it's not allowed, if just one of them is None
    if not first or not second:
        return False

    # it's not allowed, if type is different
    if not first.type == second.type:
        return False

    # find each element of the first list in the second list and each element of the second list in the first list
    if not check_list_in_list(compare_diagnosis, first.diagnoses, second.diagnoses):
        return False
    if not check_list_in_list(compare_diagnosis, second.diagnoses, first.diagnoses):
        return False

    return True


def observation_pub_cb(obs):
    observer_pub.publish(obs)


class TestDiagnosis(unittest.TestCase):
    def setUp(self):
        # pass
        global pub_tester
        pub_tester = PublisherTester("/diagnosis", diagnosis_set)

        # make publisher and wait for it
        global observer_pub
        observer_pub = rospy.Publisher('/observers/info', observer_info, queue_size=10)
        while not rospy.is_shutdown() and observer_pub.get_num_connections() < 1:
            rospy.sleep(rospy.Duration(1))

    def test_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[observation_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[observation_error]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[observation_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), 1.1)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "something is wrong :)")

    def test_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[observation_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[observation_ok]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[observation_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), 1.1)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "something is wrong :)")


if __name__ == "__main__":
    rospy.init_node('test_tug_diagnosis_node', anonymous=False)
    rostest.rosrun('tug_diagnosis', 'test_diagnosis', TestDiagnosis)
