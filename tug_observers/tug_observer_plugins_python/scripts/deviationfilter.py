#!/usr/bin/env python

from threading import Lock


class DeviationFilter():
    """
    Base class for deviation.
    """
    def __init__(self):
        pass

    def update(self, new_value):
        """
        Give the deviation a new value.
        Consider the use of resources, because this can be called very often.
        :param new_value: New value for the deviation
        """
        pass

    def get_value(self):
        """
        Return the result here.
        :return: Result of the deviation
        """
        return []

    def reset(self):
        """
        Reset the deviation, to remove any history. Necessary for a clean restart.
        """
        pass


class DeviationFilterFactory():
    """
    Factory for getting the right deviation instance.
    """
    @staticmethod
    def create_deviation_filter(config):
        """
        Decode deviation type from config and return new instance of corresponding deviation.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding deviation
        """
        type = config['deviation_type']
        if type == "min_max":
            return MinMaxDeviationFilter()
        else:
            return DeviationFilterFactory()


class MinMaxDeviationFilter(DeviationFilter):
    def __init__(self):
        DeviationFilter.__init__(self)
        self._min = float("inf")
        self._max = float("-inf")

    def update(self, new_value):
        self._min = min(self._min, new_value)
        self._max = max(self._max, new_value)

    def get_value(self):
        return [self._min, self._max]

    def reset(self):
        self._min = float("inf")
        self._max = float("-inf")

