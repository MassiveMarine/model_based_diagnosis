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
    @staticmethod
    def create_deviation_filter(config):
        """
        Decode deviation type from config and return new instance of corresponding deviation.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding deviation
        """
        type = Config.get_param(config, 'deviation_type')
        if type == "min_max":
            return MinMaxDeviationFilter(config)
        elif type == "std_deviation":
            return StdDeviationDeviationFilter(config)
        else:
            return DeviationFilter()
            # raise NameError("'" + str(type) + "' from config not found in deviation-filter!")


class StdDeviationDeviationFilter(DeviationFilter):
    def __init__(self, config):
        DeviationFilter.__init__(self)
        from collections import deque
        from math import sqrt
        self.sqrt = sqrt
        self.window_size = Config.get_param(config, 'window_size')
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        self._ring_buffer.append(new_value)

    def get_deviation(self):
        data = list(self._ring_buffer)
        n = len(data)

        if n < 2:
            return []

        c = sum(data)/n
        ss = sum((x-c)**2 for x in data)

        ss -= sum((x-c) for x in data)**2/len(data)

        variance = ss/(n-1)
        result = self.sqrt(variance)
        return [result]

    def reset(self):
        self._ring_buffer.clear()


class MinMaxDeviationFilter(DeviationFilter):
    def __init__(self, config):
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
        # print 'use unbuffered'
        self._min = min(new_value, self._min)
        self._max = max(new_value, self._max)

    def update_buffered(self, new_value):
        # print 'use buffered'
        self._ring_buffer.append(new_value)

    def get_deviation_unbuffered(self):
        return [self._min, self._max]

    def get_deviation_buffered(self):
        ring_list = list(self._ring_buffer) + [float("nan")]
        return [min(ring_list), max(ring_list)]

    def reset_unbuffered(self):
        self._min = float("nan")
        self._max = float("nan")

    def reset_buffered(self):
        self._ring_buffer.clear()

