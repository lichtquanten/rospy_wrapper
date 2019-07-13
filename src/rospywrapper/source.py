from abc import ABCMeta, abstractmethod
import rosbag
from rospy_message_converter import message_converter
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
    def __init__(self, topic, filename):
        self.topic = topic
        self.filename = filename

    def __iter__(self):
        return self

    def next(self):
        _, msg, t = next(self._messages)
        msg = message_converter.convert_ros_message_to_dictionary(msg)
        return msg, t

    def __enter__(self):
        self._bag = rosbag.Bag(self.filename, 'r')
        self._bag.__enter__()
        self._messages = self._bag.read_messages(
            connection_filter=lambda topic, *args: topic == self.topic)
        return self

    def __exit__(self, *exc):
        self._bag.__exit__(None, None, None)

class ROSLiveSource(Source):
    def __init__(self, topic, data_type):
        self.topic = topic
        self.data_type = data_type
        self._buffer = []

    def __iter__(self):
        while not rospy.is_shutdown():
            if self._buffer:
                yield self._buffer.pop(0)
            else:
                time.sleep(0.001)

    def _callback(self, msg):
        msg = message_converter.convert_ros_message_to_dictionary(msg)
        self._buffer.append((data, rospy.get_rostime()))

    def __enter__(self):
        self._subscriber = rospy.Subscriber(
            self.topic, self.data_type, self._callback)
        return self

    def __exit__(self, *exc):
        self._subscriber.unregister()
