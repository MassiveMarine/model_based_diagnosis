#!/usr/bin/env python

import rospy
from tug_python_utils import YamlHelper as Config
from scipy.special import stdtr
from numpy import abs, sqrt

class StudentTTest():
    """
    Base class for student t test.
    """
    def __init__(self, config):
        """
        Constructor for a new hypothesis by using the student t test. It includes the value,
        the deviation and the number of samples.
        :param config: Configuration from yaml file
        """
        self._true_mean = Config.get_param(config, 'true_mean')
        if Config.has_key(config, 'std_deviation'):
            self._std_deviation = Config.get_param(config, 'std_deviation')
        else:
            self._std_deviation = None
        self._significance_level = Config.get_param(config, 'significance_level')

    def check_hypothesis(self, value, deviation, sample_size):
        """
        Check if the given information fits to the defined hypothesis.
        :param value: mean value which should be checked
        :param deviation: deviation values which should be checked
        :param sample_size: number of samples that are used for mean and deviation
        :return: True if the hypothesis corroborate, otherwise False
        """
        if len(deviation) is not 1:
            # raise AttributeError('student t test needs one deviation as parameter')
            return False

        if self._std_deviation:
            deviation = self._std_deviation
        else:
            deviation = deviation[0]

        mean_difference = value - self._true_mean

        tf = mean_difference * sqrt(float(sample_size)) / deviation # t-statistic for mean

        avar = deviation**2
        na = sample_size
        adof = na - 1
        dof = (avar/na)**2 / (avar**2/(na**2*adof))

        q = stdtr(dof, -abs(tf))*2  # two-sided pvalue = Prob(abs(t)>tt)
        # rospy.loginfo('t-statistic = %6.3f pvalue = %6.10f' % (tf, q))
        # print 't-statistic = %6.3f pvalue = %6.10f' % (tf, q)
        return False if q < self._significance_level/2. else True