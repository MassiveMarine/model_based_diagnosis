#!/usr/bin/env python

from threading import Lock


class Filter():
    def __init__(self):
        self.list_lock = Lock()
        pass

    def update(self, new_value):
        pass

    def get_value(self):
        pass

    def reset(self):
        pass


class FilterFactory():
    @staticmethod
    def create_filter(config):
        type = config['type']
        if type == "mean":
            return MeanFilter(config)
        elif type == "median":
            return MedianFilter(config)
        elif type == "kmeans":
            return KMeansFilter(config)
        elif type == "lp":
            return LowPassFilter(config)
        elif type == "nofilter":
            return NoFilter()
        else:
            return None


class MedianFilter(Filter):
    def __init__(self, config):
        from collections import deque
        Filter.__init__(self)
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=10)

    def update(self, new_value):
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.list_lock.release()

    def get_value(self):

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

        self.list_lock.release()
        return result

    def reset(self):
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.list_lock.release()


class MeanFilter(Filter):
    def __init__(self, config):
        from collections import deque
        Filter.__init__(self)
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=10)

    def update(self, new_value):
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.list_lock.release()

    def get_value(self):
        self.list_lock.acquire()
        size = len(self._ring_buffer)
        if not size:
            result = None
        else:
            result = sum(self._ring_buffer) / size

        self.list_lock.release()
        return result

    def reset(self):
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.list_lock.release()


class KMeansFilter(Filter):
    def __init__(self, config):
        from collections import deque
        Filter.__init__(self)
        self.k_half = config['k_size'] / 2
        self.window_size = config['window_size']
        self._ring_buffer = deque(maxlen=10)

    def update(self, new_value):
        self.list_lock.acquire()
        self._ring_buffer.append(new_value)
        self.list_lock.release()

    def get_value(self):
        self.list_lock.acquire()

        size = len(self._ring_buffer)
        if not size:
            result = None
        else:

            center_index = size // 2
            lower_index = max(center_index - self.k_half, 0)
            upper_index = min(center_index + self.k_half, size)

            small_list = list(self._ring_buffer)[lower_index:upper_index + 1]

            result = sum(small_list) / len(small_list)

        self.list_lock.release()
        return result

    def reset(self):
        self.list_lock.acquire()
        self._ring_buffer.clear()
        self.list_lock.release()


class LowPassFilter(Filter):
    def __init__(self, config):
        Filter.__init__(self)
        self._decay_rate = config['decay_rate']
        self._current_value = None

    def update(self, new_value):
        self.list_lock.acquire()
        if self._current_value is None:
            self._current_value = new_value * 1.0
        else:
            self._current_value = self._current_value * (1.0 - self._decay_rate) + new_value * self._decay_rate
        self.list_lock.release()

    def get_value(self):
        return self._current_value

    def reset(self):
        self.list_lock.acquire()
        self._current_value = None
        self.list_lock.release()


class NoFilter(Filter):
    def __init__(self):
        Filter.__init__(self)
        self._current_value = None

    def update(self, new_value):
        self._current_value = new_value

    def get_value(self):
        return self._current_value

    def reset(self):
        self._current_value = None