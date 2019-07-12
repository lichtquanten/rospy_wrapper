from abc import ABCMeta, abstractmethod
import rosbag
from rospy_message_converter import message_converter
import Queue
import time
import rospy

class Source(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def __iter__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class ROSBagSource(Source):
    def __init__(self, preprocess, filename, topic):
        self.preprocess = preprocess
        self._filename = filename
        self._topic = topic
        self.start_time = None
        self.rate = 25
        self.chunk_duration = 1 / float(self.rate)

    def __iter__(self):
        return self

    def next(self):
        topic, msg, t = next(self._messages)
        msg = message_converter.convert_ros_message_to_dictionary(msg)
        start = msg['header']['stamp']['secs'] + msg['header']['stamp']['nsecs'] * (10 ** -9)
        return self.preprocess(msg), start, start + self.chunk_duration

    def __enter__(self):
        self._bag = rosbag.Bag(self._filename, 'r')
        self._bag.__enter__()
        self.start_time = self._bag.get_start_time()
        self._messages = self._bag.read_messages(
            connection_filter=lambda topic, *args: topic == self._topic)
        return self

    def __exit__(self, *exc):
        self._bag.__exit__(None, None, None)

class ROSLiveSource(Source):
    def __init__(self, preprocess, topic, msg_type, rate, start_time=None):
        self.preprocess = preprocess
        self._topic = topic
        self._type = msg_type
        self.rate = rate
        self._buffer = []
        if start_time is None:
            t = rospy.Time.now()
            self.start_time = t.secs + t.nsecs / (10 ** 9)
        else:
            self.start_time = start_time

    def __iter__(self):
        while not rospy.is_shutdown():
            if self._buffer:
                yield self._buffer.pop(0)
            else:
                time.sleep(0.001)

    def _callback(self, msg):
        msg = message_converter.convert_ros_message_to_dictionary(msg)
        start_time = msg['time']['secs'] - self.start_time + (float(msg['time']['nsecs']) / (10 ** 9))
        data = self.preprocess(msg)
        self._buffer.append((data, start_time, start_time + len(data) / float(self.rate)))

    def __enter__(self):
        self._subscriber = rospy.Subscriber(
            self._topic, self._type, self._callback)
        return self

    def __exit__(self, *exc):
        self._subscriber.unregister()
