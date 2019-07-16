from abc import ABCMeta, abstractmethod
import rosbag
import rospy

class Sink(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def put(self, topic, data_class, data, t):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class ROSTopicSink(Sink):
    def __init__(self):
        self._publishers = {}

    def put(self, topic, data_class, data, t):
        if topic not in self._publishers:
            self._publishers[topic] = rospy.Publisher(topic, data_class)
        self._publishers[topic].publish(data)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        for _, publisher in self._publishers:
            publisher.unregister()
        self._publishers = {}

class ROSBagSink(Sink):
    def __init__(self, filename):
        self.filename = filename
        self._bag = None

    def __enter__(self):
        self._bag = rosbag.Bag(self.filename, 'w')
        self._bag.__enter__()
        return self

    def __exit__(self, *exc):
        self._bag.__exit__(None, None, None)
        self._bag = None

    def put(self, topic, data_class, data, t):
        self._bag.write(topic, data_class(data), t)
