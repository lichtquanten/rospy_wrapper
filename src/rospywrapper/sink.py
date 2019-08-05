from abc import ABCMeta, abstractmethod
import genpy
import rosbag
import rospy

class Sink(object):
    """Base class for sink objects.

    Subclasses should implement method to receive data.
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def put(self, data, t):
        """Write data to the sink.

        Args:
            data: Data to be written to sink.
            t (rospy.time.Time): A timestamp for `data`.
        """
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class TopicSink(Sink):
    """A class for publishing data to ROS topics."""

    def __init__(self, topic, data_class, queue_size=None):
        """Constructor.
        Args:
            topic (str): The resource name of a topic.
            data_class (genpy.Message): Messsage class for serialization of `data.`
            queue_size (int, optional): `queue_size` of rospy publisher.
        """
        self._topic = topic
        self._data_class = data_class
        self._queue_size = queue_size

    def put(self, data, t):
        """Publish data to the topic specified in the constructor.

        Args:
            data: An acceptable input to the `data_class` constructor, where
                `data_class` is given in the constructor.
            t (rospy.time.Time): A timestamp for `data`.
        """
        if type(data) is dict:
            msg = self._data_class(**data)
        else:
            msg = self._data_class(data)
        self._publisher.publish(msg)

    def __enter__(self):
        """Register the publisher."""
        if self._queue_size is not None:
            self._publisher = rospy.Publisher(self._topic, self._data_class,
            queue_size=self._queue_size)
        else:
            self._publisher = rospy.Publisher(self._topic, self._data_class)
        return self

    def __exit__(self, *exc):
        """Unregister the publisher."""
        self._publisher.unregister()

class BagSink(Sink):
    """Class for writing data to a bag file."""

    def __init__(self, bag, topic, data_class):
        """
        Args:
            bag (rosbag.bag.Bag): A `bag` object with write permissions.
            topic (str): The resource name of a topic.
            data_class (genpy.Message): Messsage class for serialization.
        """
        self._bag = bag
        self._topic = topic
        self._data_class = data_class

    def put(self, data, t):
        """Write `data` to the bag at time `t`.

        Args:
            data: An acceptable input to the `data_class` constructor, where
                `data_class` is given in the constructor.
            t (rospy.time.Time): A timestamp for `data`.
        """
        if type(data) is dict:
            msg = self._data_class(**data)
        else:
            msg = self._data_class(data)
        self._bag.write(self._topic, msg, t)
