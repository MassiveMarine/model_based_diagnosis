#!/usr/bin/env python

import rospy

from nominal_value import NominalValueFactory
from student_t_test import StudentTTest
from tug_python_utils import YamlHelper as Config


class SingleValueHypothesisCheck():
    """
    Base class for single value hypothesis check.
    """
    def __init__(self):
        pass

    def check_hypothesis(self, value, deviation=[], sample_size=0):
        """
        Check the hypothesis here and return the result.
        :param value: mean value which should be checked
        :param deviation: deviation values which should be checked
        :param sample_size: number of samples that are used for mean and deviation
        :return: True if the hypothesis corroborate, otherwise False
        """
        rospy.logerr("SingleValueHypothesisCheck not implemented")
        return False


class SingleValueHypothesisCheckFactory():
    """
    Factory for getting the right hypothesis check instance.
    """

    def __init__(self):
        pass

    @staticmethod
    def create_single_value_hypothesis_check(config):
        """
        Decode single value hypothesis type from config and return new instance of corresponding hypothesis check.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding hypothesis check
        """
        hypothesis_type = Config.get_param(config, 'type')

        if hypothesis_type in ['gauss', 'exact', 'not', 'greather_than', 'less_than', 'in_between', 'not_in_between']:
            return NominalValueHypothesis(config)
        elif hypothesis_type == 'student_t':
            return StudentTTest(config)
        else:
            rospy.logwarn("single value hypothesis check type '" + str(hypothesis_type) + "' not found")
            return SingleValueHypothesisCheck()


class NominalValueHypothesis(SingleValueHypothesisCheck):
    """
    Nominal hypothesis check. Check if value and deviation are in defined range
    (gauss, exact, exact not, greater than, less than, in between, not in between)
    """
    def __init__(self, config):
        """
        Constructor for a new nominal value hypothesis. It includes the value and the deviation(s)
        :param config: Configuration from yaml file
        """
        SingleValueHypothesisCheck.__init__(self)
        self._value_check = NominalValueFactory.create_nominal_value(config)
        self._deviation_checks = []
        
        if not Config.has_key(config, 'deviation'):
            return

        for deviation_config in Config.get_param(config, 'deviation'):
            self._deviation_checks.append(NominalValueFactory.create_nominal_value(deviation_config))

    def check_hypothesis(self, value, deviation=[], sample_size=0):
        """
        Check if the given information fits to the defined hypothesis.
        :param value: mean value which should be checked
        :param deviation: deviation values which should be checked
        :param sample_size: number of samples that are used for mean and deviation
        :return: True if the hypothesis corroborate, otherwise False
        """
        value_nominal = self._value_check.check_hypothesis(value)
        if not len(deviation):
            return False

        if len(deviation) is not len(self._deviation_checks):
            rospy.logwarn("Number of deviations do not match with config!")
            return value_nominal

        deviation_nominal = all(o.check_hypothesis(deviation[c]) for c, o in enumerate(self._deviation_checks))

        return value_nominal and deviation_nominal
