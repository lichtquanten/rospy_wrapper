from abc import ABCMeta, abstractmethod
import csv
import rospy

class Sink(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def put(self, data, t):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class CSVSink(Sink):
    def __init__(self, filename, fieldnames):
        self.filename = filename
        self.fieldnames = fieldnames
        self.file = None
        self.writer = None

    def put(self, data, t):
        row = [t] + [data[field] for field in self.fieldnames]
        self.writer.writerow(row)

    def __enter__(self):
        self.file = open(self.filename, 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time'] + self.fieldnames)
        return self

    def __exit__(self, *exc):
        self.file.close()

class ROSLiveSink(Sink):
    def __init__(self, topic, data_type):
        self.topic = topic
        self.data_type = data_type
        self._publisher = None

    def put(self, data, t):
        self._publisher.publish(data)

    def __enter__(self):
        self._publisher = rospy.Publisher(self.topic, self.data_type)
        return self

    def __exit__(self, *exc):
        self._publisher.unregister()
