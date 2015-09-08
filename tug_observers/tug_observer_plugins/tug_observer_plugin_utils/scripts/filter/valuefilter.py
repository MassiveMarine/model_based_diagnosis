#!/usr/bin/env python

from tug_python_utils import YamlHelper as Config


class ValueFilter():
    """
    Base class for filter.
    """
    def __init__(self):
        self.sample_size = 0
        pass

    def update(self, new_value):
        """
        Give the filter a new value. Maybe its necessary to apply the filter each time.
        Consider the use of resources, because this can be called very often.
        :param new_value: New value for the filter
        """
        pass

    def get_value(self):
        """
        Return the result here. Maybe its necessary to apply the filter first.
        :return: Result of the filter
        """
        return None, None

    def reset(self):
        """
        Reset the filter, to remove any history. Necessary for a clean restart.
        """
        pass


class ValueFilterFactory():
    """
    Factory for getting the right filter instance.
    """

    def __init__(self):
        pass

    @staticmethod
    def create_value_filter(config):
        """
        Decode filter type from config and return new instance of corresponding filter.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding filter
        """
        value_filter_type = Config.get_param(config, 'type')
        if value_filter_type == "mean":
            return MeanValueFilter(config)
        elif value_filter_type == "median":
            return MedianValueFilter(config)
        elif value_filter_type == "kmeans":
            return KMeansValueFilter(config)
        elif value_filter_type == "ewma":
            return ExponentiallyWeightedMovingAverageValueFilter(config)
        elif value_filter_type == "nofilter":
            return NoValueFilter(config)
        else:
            return ValueFilter()
            # raise NameError("'" + str(value_filter_type) + "' from config not found in value-filter!")


class MedianValueFilter(ValueFilter):
    """
    Filter for getting the median of a defined number of values
    """
    def __init__(self, config):
        """
        Constructor for median filter object. Uses a ringbuffer of given size.
        :param config: Configuration from yaml file
        """
        from collections import deque
        ValueFilter.__init__(self)
        self.window_size = Config.get_param(config, 'window_size')
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self._ring_buffer.append(new_value)
        self.sample_size = len(self._ring_buffer)

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        data = sorted(self._ring_buffer)
        n = len(data)
        if n == 0:
            result = None
        elif n % 2 == 1:
            result = data[n//2]
        else:
            i = n//2
            result = (data[i - 1] + data[i])/2

        return result, self.sample_size

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self._ring_buffer.clear()
        self.sample_size = 0


class MeanValueFilter(ValueFilter):
    """
    Filter for getting the mean of a defined number of values
    """
    def __init__(self, config):
        """
        Constructor for mean filter object. Uses a ringbuffer of given size.
        :param config: Configuration from yaml file
        """
        from collections import deque
        ValueFilter.__init__(self)
        self.window_size = Config.get_param(config, 'window_size')
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self._ring_buffer.append(new_value)
        self.sample_size = len(self._ring_buffer)

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        size = len(self._ring_buffer)
        if not size:
            result = None
        else:
            result = sum(self._ring_buffer) / size

        return result, self.sample_size

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self._ring_buffer.clear()
        self.sample_size = 0


class KMeansValueFilter(ValueFilter):
    """
    Filter for getting the k-means of a defined number of values in a defined buffer.
    """
    def __init__(self, config):
        """
        Constructor for k-mean filter object. Uses a ringbuffer of given size and take the mean of window.
        :param config: Configuration from yaml file
        """
        from collections import deque
        ValueFilter.__init__(self)
        self.k_half = Config.get_param(config, 'k_size') / 2
        self.window_size = Config.get_param(config, 'window_size')
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self._ring_buffer.append(new_value)
        self.sample_size = len(self._ring_buffer)

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        size = len(self._ring_buffer)
        if not size:
            result = None
        else:
            center_index = size // 2
            lower_index = max(center_index - self.k_half, 0)
            upper_index = min(center_index + self.k_half, size)

            small_list = list(self._ring_buffer)[lower_index:upper_index + 1]

            result = sum(small_list) / len(small_list)

        return result, self.sample_size

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self._ring_buffer.clear()
        self.sample_size = 0


class ExponentiallyWeightedMovingAverageValueFilter(ValueFilter):
    """
    Filter for applying weighted values to history.
    """
    def __init__(self, config):
        """
        Constructor for exponentially weighted moving average (EWMA) filter object.
        Applies new values weighted to history.
        :param config: Configuration from yaml file
        """
        ValueFilter.__init__(self)
        self._decay_rate = Config.get_param(config, 'decay_rate')
        if Config.has_key(config, 'window_size'):
            from collections import deque
            self.window_size = Config.get_param(config, 'window_size')
            self._ring_buffer = deque(maxlen=self.window_size)
            self.update = self.update_buffered
            self.get_value = self.get_value_buffered
            self.reset = self.reset_buffered
        else:
            self._current_value = None
            self.update = self.update_unbuffered
            self.get_value = self.get_value_unbuffered
            self.reset = self.reset_unbuffered

    def update_unbuffered(self, new_value):
        """
        Add new value to the history with given weight.
        :param new_value: New value, which should be added to history
        """
        if self._current_value is None:
            self._current_value = new_value * 1.0
        else:
            self._current_value = self._current_value * (1.0 - self._decay_rate) + new_value * self._decay_rate

        self.sample_size += 1

    def update_buffered(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self._ring_buffer.append(new_value)
        self.sample_size = len(self._ring_buffer)

    def get_value_unbuffered(self):
        """
        Filter result is already up-to-date. Just return the value.
        :return: Result of the filter
        """
        return self._current_value, self.sample_size

    def get_value_buffered(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        if not len(self._ring_buffer):
            return None, self.sample_size
        value = self._ring_buffer[0]
        for index in range(1, len(self._ring_buffer)):
            value = value * (1-self._decay_rate) + self._ring_buffer[index] * self._decay_rate

        return value, self.sample_size

    def reset_unbuffered(self):
        """
        Reset the filter, that cleans the history.
        """
        self._current_value = None
        self.sample_size = 0

    def reset_buffered(self):
        """
        Reset the filter, that cleans the history.
        """
        self._ring_buffer.clear()
        self.sample_size = 0


class NoValueFilter(ValueFilter):
    """
    This is not a filter. it just stores the current value.
    """
    def __init__(self, config):
        ValueFilter.__init__(self)
        self._current_value = None

    def update(self, new_value):
        """
        Store new value as current value.
        :param new_value: New value, which should be stored
        """
        self._current_value = new_value
        self.sample_size = 1

    def get_value(self):
        """
        This is not really a filter. Just return the current value.
        :return: current stored value
        """
        return self._current_value, self.sample_size

    def reset(self):
        """
        Reset the filter, that cleans the current value.
        """
        self._current_value = None
        self.sample_size = 0
