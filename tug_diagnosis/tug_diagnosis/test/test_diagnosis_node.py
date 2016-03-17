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
global max_diagnosis_timeout
max_diagnosis_timeout = 0.5


obs_ok = observation(observation_msg='ok', verbose_observation_msg='everything ok', observation=1)
obs_error = observation(observation_msg='fail', verbose_observation_msg='something failed', observation=-1)


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


def check_list_in_list(compare_fct, first, second, enable_equal_check=False):
    # find each element of the first list in the second list
    for entry_of_first in first:
        first_found_in_second = False
        for entry_of_second in second:
            if compare_fct(entry_of_first, entry_of_second):
                first_found_in_second = True
                break
        if not first_found_in_second:
            return False
    if enable_equal_check:
        return check_list_in_list(compare_fct, second, first)
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
    if not check_list_in_list(compare_resource_mode_assignement, first.diagnosis, second.diagnosis,
                              enable_equal_check=True):
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
    if not check_list_in_list(compare_diagnosis, first.diagnoses, second.diagnoses, enable_equal_check=True):
        return False

    return True


def observation_pub_cb(obs):
    observer_pub.publish(obs)


class TestDiagnosis(unittest.TestCase):
    def setUp(self):
        global pub_tester
        pub_tester = PublisherTester("/diagnosis", diagnosis_set)

        # make publisher and wait for it
        global observer_pub
        observer_pub = rospy.Publisher('/observers/info', observer_info, queue_size=10)
        while not rospy.is_shutdown() and observer_pub.get_num_connections() < 1:
            rospy.sleep(rospy.Duration(1))

# ########################################################################################################################
# #                                        test diagnosis with 'hz' observation                                        #
# ########################################################################################################################
    def test_hz_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_hz_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='hz', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

########################################################################################################################
#                                        test diagnosis with 'score' observation                                       #
########################################################################################################################
    def test_scores_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_error]),
            observation_info(header=header, type='hz', resource=str('/topic1 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_error]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)

        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_scores_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='scores', resource=str('/topic1'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic2'), observation=[obs_ok]),
            observation_info(header=header, type='scores', resource=str('/topic3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")


########################################################################################################################
#                                     test diagnosis with 'timestamp' observation                                      #
########################################################################################################################
    def test_timestamp_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timestamp_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timestamp', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timestamp', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

########################################################################################################################
#                                     test diagnosis with 'timeout' observation                                      #
########################################################################################################################
    def test_timeout_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_error]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timeout_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timeout', resource=str('/topic1 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic2 []'), observation=[obs_ok]),
            observation_info(header=header, type='timeout', resource=str('/topic3 []'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

########################################################################################################################
#                                     test diagnosis with 'resources' observation                                      #
########################################################################################################################
    def test_resources_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_resources_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='resources', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='resources', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

########################################################################################################################
#                                     test diagnosis with 'activated' observation                                      #
########################################################################################################################
    def test_activated_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_2(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_3(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3'),
                                                       resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_4(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node3')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_5(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_6(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_7(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_8(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node3')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_9(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_error]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1'),
                                                       resource_mode_assignment_error('node2'),
                                                       resource_mode_assignment_error('node3')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_activated_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='activated', resource=str('node1'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node2'), observation=[obs_ok]),
            observation_info(header=header, type='activated', resource=str('node3'), observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")


########################################################################################################################
#                                     test diagnosis with 'timing' observation                                      #
########################################################################################################################
    def test_timing_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timing', resource=str('/topic1 [] /topic2 []'),
                             observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_timing_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='timing', resource=str('/topic1 [] /topic2 []'),
                             observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

########################################################################################################################
#                                     test diagnosis with 'velocity' observation                                      #
########################################################################################################################
    def test_velocity_1(self):
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='velocity', resource=str('/topic1 /topic2'),
                             observation=[obs_error])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = diagnosis_set()
        expected_diagnosis.type = 'hsdag-ci-picosat'
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node1')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('node2')]))
        expected_diagnosis.diagnoses.append(diagnosis([resource_mode_assignment_error('velocity')]))

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")

    def test_velocity_999(self):
        # not only a test! need to be the highest number to be run as last to clean set used observations to 'OK'
        global pub_tester
        header = Header(stamp=rospy.Time.now())
        observation_data = [
            observation_info(header=header, type='velocity', resource=str('/topic1 /topic2'),
                             observation=[obs_ok])]
        msg = observer_info(observation_infos=observation_data)

        expected_diagnosis = None

        received_diagnosis = pub_tester.get_message(lambda: observation_pub_cb(msg), max_diagnosis_timeout)
        self.assertTrue(compare_diagnosis_set(received_diagnosis, expected_diagnosis), "Diagnosis is not correct!")


if __name__ == "__main__":
    rospy.init_node('test_tug_diagnosis_node', anonymous=False)

    rostest.rosrun('tug_diagnosis', 'test_diagnosis', TestDiagnosis)
