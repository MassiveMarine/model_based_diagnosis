#!/usr/bin/env python

from tug_python_utils import YamlHelper as Config


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

    def get_deviation(self):
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

    def __init__(self):
        pass

    @staticmethod
    def create_deviation_filter(config):
        """
        Decode deviation type from config and return new instance of corresponding deviation.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding deviation
        """
        deviation_filter_type = Config.get_param_with_default(config, 'deviation_type', 'unknown')
        if deviation_filter_type == "min_max":
            return MinMaxDeviationFilter(config)
        elif deviation_filter_type == "std_deviation":
            return StdDeviationDeviationFilter(config)
        else:
            return DeviationFilter()
            # raise NameError("'" + str(deviation_filter_type) + "' from config not found in deviation-filter!")


class StdDeviationDeviationFilter(DeviationFilter):
    """
    Class for calculation the standard deviation. It can be used with or without fixed buffer size.
    """
    def __init__(self, config):
        """
        Constructor to create StdDeviationFilter object. It reads and setup
        the buffer (defined or not defined buffer size)
        :param config: Configuration from yaml file
        """
        DeviationFilter.__init__(self)
        from math import sqrt
        self.sqrt = sqrt

        if Config.has_key(config, 'window_size'):
            from collections import deque
            self.window_size = Config.get_param(config, 'window_size')
            self._buffer = deque(maxlen=self.window_size)
            self.reset = self.reset_buffered
        else:
            self._buffer = []
            self.reset = self.reset_unbuffered

    def update(self, new_value):
        """
        Append new value to buffer.
        :param new_value: New value, which should be added to buffer
        """
        self._buffer.append(new_value)

    def get_deviation(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        data = list(self._buffer)
        n = len(data)

        if n < 2:
            return []

        c = sum(data)/n
        ss = sum((x-c)**2 for x in data)

        ss -= sum((x-c) for x in data)**2/len(data)

        variance = ss/(n-1)
        result = self.sqrt(variance)
        return [result]

    def reset_unbuffered(self):
        """
        Reset the filter, that cleans the buffer if unbuffered.
        """
        self._buffer = []

    def reset_buffered(self):
        """
        Reset the filter, that cleans the buffer if buffered.
        """
        self._buffer.clear()


class MinMaxDeviationFilter(DeviationFilter):
    """
    Class for calculation the min-max deviation. It can be used with or without fixed buffer size.
    """
    def __init__(self, config):
        """
        Constructor to create min-max deviation object. It reads and setup
        the buffer (defined or not defined buffer size)
        :param config: Configuration from yaml file
        """
        DeviationFilter.__init__(self)

        if Config.has_key(config, 'window_size'):
            from collections import deque
            self.window_size = Config.get_param(config, 'window_size')
            self._ring_buffer = deque(maxlen=self.window_size)
            self.update = self.update_buffered
            self.get_deviation = self.get_deviation_buffered
            self.reset = self.reset_buffered
        else:
            self._min = float("nan")
            self._max = float("nan")
            self.update = self.update_unbuffered
            self.get_deviation = self.get_deviation_unbuffered
            self.reset = self.reset_unbuffered

    def update_unbuffered(self, new_value):
        """
        Check if new value is a new max or new min.
        :param new_value: New value, which should be checked
        """
        self._min = min(new_value, self._min)
        self._max = max(new_value, self._max)

    def update_buffered(self, new_value):
        """
        Append new value to buffer.
        :param new_value: New value, which should be added to buffer
        """
        self._ring_buffer.append(new_value)

    def get_deviation_unbuffered(self):
        """
        Result is the already known min and max value.
        :return: Result of the applied filter
        """
        return [self._min, self._max]

    def get_deviation_buffered(self):
        """
        Find min and max in the buffer.
        :return: Result of min and max
        """
        ring_list = list(self._ring_buffer) + [float("nan")]
        return [min(ring_list), max(ring_list)]

    def reset_unbuffered(self):
        """
        Reset the filter, that cleans min and max.
        """
        self._min = float("nan")
        self._max = float("nan")

    def reset_buffered(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self._ring_buffer.clear()
