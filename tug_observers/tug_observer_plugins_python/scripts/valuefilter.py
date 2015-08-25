#!/usr/bin/env python

from threading import Lock

from deviationfilter import DeviationFilterFactory


class ValueFilter():
    """
    Base class for filter.
    """
    def __init__(self, config):
        self.list_lock = Lock()
        self.deviation = DeviationFilterFactory.create_deviation_filter(config)
        self.sample_size = 0

    def update(self, new_value):
        """
        Give the filter a new value. Maybe its necessary to apply the filter each time.
        Consider the use of resources, because this can be called very often.
        :param new_value: New value for the filter
        """
        self.deviation.update(new_value)
        pass

    def get_value(self):
        """
        Return the result here. Maybe its necessary to apply the filter first.
        :return: Result of the filter
        """
        return None, self.deviation.get_value()

    def reset(self):
        """
        Reset the filter, to remove any history. Necessary for a clean restart.
        """
        self.deviation.reset()
        self.sample_size = 0
        pass

    def get_sample_size(self):
        return self.sample_size

    def increment_sample_size(self, increment):
        self.sample_size += increment

    def set_sample_size(self, new_sample_size):
        self.sample_size = new_sample_size


class ValueFilterFactory():
    """
    Factory for getting the right filter instance.
    """
    @staticmethod
    def create_value_filter(config):
        """
        Decode filter type from config and return new instance of corresponding filter.
        :param config: Configuration from yaml file
        :return: New instance of a corresponding filter
        """
        type = config['type']
        if type == "mean":
            return MeanValueFilter(config)
        elif type == "median":
            return MedianValueFilter(config)
        elif type == "kmeans":
            return KMeansValueFilter(config)
        elif type == "ewma":
            return ExponentiallyWeightedMovingAverageValueFilter(config)
        elif type == "nofilter":
            return NoValueFilter(config)
        else:
            return None


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
        ValueFilter.__init__(self, config)
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=self.window_size)
        self.deviation_type = config['min_max']

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.deviation.update(new_value)
        self.list_lock.release()

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        self.list_lock.acquire()
        data = sorted(self._ring_buffer)
        n = len(data)
        if n == 0:
            result = None
        elif n%2 == 1:
            result = data[n//2]
        else:
            i = n//2
            result = (data[i - 1] + data[i])/2
        deviation = self.deviation.get_value()

        self.list_lock.release()
        return result, deviation

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.deviation.reset()
        self.list_lock.release()


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
        ValueFilter.__init__(self, config)
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.deviation.update(new_value)
        self.list_lock.release()

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        self.list_lock.acquire()
        size = len(self._ring_buffer)
        if not size:
            result = None
            deviation = []
        else:
            result = sum(self._ring_buffer) / size
            deviation = self.deviation.get_value()

        self.list_lock.release()
        return result, deviation

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.deviation.reset()
        self.list_lock.release()


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
        ValueFilter.__init__(self, config)
        self.k_half = config['k_size'] / 2
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=self.window_size)

    def update(self, new_value):
        """
        Append new value to ringbuffer.
        :param new_value: New value, which should be added to ring buffer
        """
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.deviation.update(new_value)
        self.list_lock.release()

    def get_value(self):
        """
        Apply the filter and return the resulting value.
        :return: Result of the applied filter
        """
        self.list_lock.acquire()

        size = len(self._ring_buffer)
        if not size:
            result = None
            deviation = []
        else:

            center_index = size // 2
            lower_index = max(center_index - self.k_half, 0)
            upper_index = min(center_index + self.k_half, size)

            small_list = list(self._ring_buffer)[lower_index:upper_index + 1]

            result = sum(small_list) / len(small_list)
            deviation = self.deviation.get_value()

        self.list_lock.release()
        return result, deviation

    def reset(self):
        """
        Reset the filter, that cleans the buffer.
        """
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.deviation.reset()
        self.list_lock.release()


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
        ValueFilter.__init__(self, config)
        self._decay_rate = config['decay_rate']
        self._current_value = None

    def update(self, new_value):
        """
        Add new value to the history with given weight.
        :param new_value: New value, which should be added to history
        """
        self.list_lock.acquire()
        if self._current_value is None:
            self._current_value = new_value * 1.0
        else:
            self._current_value = self._current_value * (1.0 - self._decay_rate) + new_value * self._decay_rate

        self.deviation.update(new_value)
        self.list_lock.release()

    def get_value(self):
        """
        Filter result is already up-to-date. Just return the value.
        :return: Result of the filter
        """
        return self._current_value, self.deviation.get_value()

    def reset(self):
        """
        Reset the filter, that cleans the history.
        """
        self.list_lock.acquire()
        self._current_value = None
        self.deviation.reset()
        self.list_lock.release()


class NoValueFilter(ValueFilter):
    """
    This is not a filter. it just stores the current value.
    """
    def __init__(self, config):
        ValueFilter.__init__(self, config)
        self._current_value = None

    def update(self, new_value):
        """
        Store new value as current value.
        :param new_value: New value, which should be stored
        """
        self._current_value = new_value
        self.deviation.update(new_value)

    def get_value(self):
        """
        This is not really a filter. Just return the current value.
        :return: current stored value
        """
        return self._current_value, self.deviation.get_value()

    def reset(self):
        """
        Reset the filter, that cleans the current value.
        """
        self._current_value = None
        self.deviation.reset()
