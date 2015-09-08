#!/usr/bin/env python

import rospy
from tug_python_utils import YamlHelper as Config

class NominalValue():
    """
    Base class for nominal value.
    """
    def __init__(self):
        pass

    def check_hypothesis(self, value):
        """
        Should contain the verification of the value based on the defined limits.
        :param value: Input that should be checked
        :return: True if value correspond to limits, otherwise False
        """
        return False
        pass


class NominalValueFactory():
    """
    Factory for getting the right verification instance.
    """
    @staticmethod
    def create_nominal_value(config):
        """
        Decode verification type and return new corresponding object.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding verification object
        """
        type = Config.get_param(config, 'type')
        if type == "gauss":
            return GaussNominalValue(config)
        elif type == "exact":
            return ExactValue(config)
        elif type == "not":
            return NotValue(config)
        elif type == "greather_than":
            return GreaterThanValue(config)
        elif type == "less_than":
            return LessThanValue(config)
        elif type == "in_between":
            return InBetweenValue(config)
        elif type == "not_in_between":
            return NotInBetweenValue(config)
        else:
            rospy.logwarn("nominal value type '" + str(type) + "' not found")
            # return None
            return NominalValue()


class GaussNominalValue(NominalValue):
    """
    Gaussian verification of value.
    """
    def __init__(self, config):
        """
        Constructor for gaussian verification. Uses mean and standard deviation.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._mean = Config.get_param(config, 'mean')
        self._std_deviation = Config.get_param(config, 'std_deviation')

    def _distance_to_mean(self, value):
        """
        Calculates the difference of the given value and the mean of the Gauss.
        :param value: Value, from which the difference to mean should be calculated
        :return: difference of the given value and the mean of the Gauss.
        """
        if value < self._mean:
            return abs(self._mean - value)
        return abs(value - self._mean)

    def check_hypothesis(self, value):
        """
        Check if the given value confirms with the defined Gauss.
        :param value: Value that should be checked
        :return: True if value confirms with the defined Gauss, otherwise False
        """
        distance = self._distance_to_mean(value)
        return True if distance < self._std_deviation else False


class ExactValue(NominalValue):
    """
    Check if value is the same as the defined one.
    """
    def __init__(self, config):
        """
        Constructor for exact verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._exact = Config.get_param(config, 'exact')

    def check_hypothesis(self, value):
        """
        Check if given value is exactly the same as defined.
        :param value: Value that should be checked
        :return: True if value confirms with the given requirements, otherwise False
        """
        return True if value is self._exact else False


class NotValue(NominalValue):
    """
    Check if value is not the same as the defined one.
    """
    def __init__(self, config):
        """
        Constructor for exactly not verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._exact_not = Config.get_param(config, 'exact_not')

    def check_hypothesis(self, value):
        """
        Check if given value is exactly not the same as defined.
        :param value: Value that should be checked
        :return: True if value confirms not with the given requirements, otherwise False
        """
        return True if value is not self._exact_not else False


class GreaterThanValue(NominalValue):
    """
    Check if value is greater than the defined one.
    """
    def __init__(self, config):
        """
        Constructor for greater than verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._greater_than = Config.get_param(config, 'greater_than')

    def check_hypothesis(self, value):
        """
        Check if given value is greater than defined.
        :param value: Value that should be checked
        :return: True if value confirms with the given requirements, otherwise False
        """
        return True if value > self._greater_than else False


class LessThanValue(NominalValue):
    """
    Check if value is less than the defined one.
    """
    def __init__(self, config):
        """
        Constructor for less than verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._less_than = Config.get_param(config, 'less_than')

    def check_hypothesis(self, value):
        """
        Check if given value is less than defined.
        :param value: Value that should be checked
        :return: True if value confirms with the given requirements, otherwise False
        """
        return True if value < self._less_than else False


class InBetweenValue(NominalValue):
    """
    Check if value is greater than a defined lower bound and smaller than a upper bound.
    """
    def __init__(self, config):
        """
        Constructor for in between verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._lower_bound = Config.get_param(config, 'lower_bound')
        self._upper_bound = Config.get_param(config, 'upper_bound')
        if self._lower_bound > self._lower_bound:
            rospy.logwarn("lower bound is bigger than upper bound. 'InBetweenValue' will not work correctly!")

    def check_hypothesis(self, value):
        """
        Check if given value is greater than a lower bound and smaller than a upper bound.
        :param value: Value that should be checked
        :return: True if value confirms with the given requirements, otherwise False
        """
        return True if self._lower_bound < value < self._upper_bound else False


class NotInBetweenValue(NominalValue):
    """
    Check if value is smaller than a defined lower bound and greater than a upper bound.
    """
    def __init__(self, config):
        """
        Constructor for not in between verification.
        :param config: Configuration from yaml file
        """
        NominalValue.__init__(self)
        self._lower_bound = Config.get_param(config, 'lower_bound')
        self._upper_bound = Config.get_param(config, 'upper_bound')
        if self._lower_bound > self._lower_bound:
            rospy.logwarn("lower bound is bigger than upper bound. 'NotInBetweenValue' will not work correctly!")

    def check_hypothesis(self, value):
        """
        Check if given value is smaller than a lower bound and greater than a upper bound.
        :param value: Value that should be checked
        :return: True if value confirms with the given requirements, otherwise False
        """
        return True if value < self._lower_bound or self._upper_bound < value else False