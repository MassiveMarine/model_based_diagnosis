#!/usr/bin/env python

import rospy


class NominalValue():
    def __init__(self):
        pass

    def is_nominal(self):
        pass


class NominalValueFactory():
    @staticmethod
    def create_nominal_value(config):
        print config
        type = config['type']
        if type == "gaus":
            return GausNominalValue(config)
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
        else:
            rospy.logwarn("nominal value type '" + str(type) + "' not found")
            return None


class GausNominalValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._mean = config['mean']
        self._std_deviation = config['std_deviation']

    def _distance_to_mean(self, value):
        if value < self._mean:
            return abs(self._mean - value)
        return abs(value - self._mean)

    def is_nominal(self, value):
        distance = self._distance_to_mean(value)
        # rospy.logerr("got distance to mean " + str(distance) + " allowed is " + str(self._std_deviation))
        return True if distance < self._std_deviation else False


class ExactValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._exact = config['exact']

    def is_nominal(self, value):
        return True if value is self._exact else False


class NotValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._exact_not = config['exact_not']

    def is_nominal(self, value):
        return True if value is not self._exact_not else False


class GreaterThanValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._greater_than = config['greater_than']

    def is_nominal(self, value):
        return True if value > self._greater_than else False


class LessThanValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._less_than = config['less_than']

    def is_nominal(self, value):
        return True if value < self._less_than else False


class InBetweenValue(NominalValue):
    def __init__(self, config):
        NominalValue.__init__(self)
        self._lower_bound = config['lower_bound']
        self._upper_bound = config['upper_bound']

    def is_nominal(self, value):
        return True if self._lower_bound < value < self._upper_bound else False