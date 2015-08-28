#!/usr/bin/env python

from threading import Lock

from deviationfilter import DeviationFilterFactory
from valuefilter import ValueFilterFactory


class Filter():
    """
    Base class for filter.
    """
    def __init__(self, config):
        self.list_lock = Lock()
        self.value_filter = ValueFilterFactory.create_value_filter(config)
        self.deviation_filter = DeviationFilterFactory.create_deviation_filter(config)
        self.sample_size = 0

    def update(self, new_value):
        """
        Give the filter a new value. Maybe its necessary to apply the filter each time.
        Consider the use of resources, because this can be called very often.
        :param new_value: New value for the filter
        """
        self.list_lock.acquire()
        self.value_filter.update(new_value)
        self.deviation_filter.update(new_value)
        self.list_lock.release()

    def get_value(self):
        """
        Return the result here. Maybe its necessary to apply the filter first.
        :return: Result of the filter
        """
        self.list_lock.acquire()
        value = self.value_filter.get_value()
        deviation = self.deviation_filter.get_deviation()
        self.list_lock.release()
        return value, deviation

    def reset(self):
        """
        Reset the filter, to remove any history. Necessary for a clean restart.
        """
        self.list_lock.acquire()
        self.value_filter.reset()
        self.deviation_filter.reset()
        self.sample_size = 0
        self.list_lock.release()

    def get_sample_size(self):
        return self.sample_size

    def increment_sample_size(self, increment):
        self.sample_size += increment

    def set_sample_size(self, new_sample_size):
        self.sample_size = new_sample_size
