import rospy
import timeout_decorator
import unittest

from rospywrapper import TopicSource, TopicSink

PKG = 'rospy_wrapper'
NAME = 'test_topic'
TIMEOUT = 20

class TestTopic(unittest.TestCase):

    @timeout_decorator.timeout(TIMEOUT)
    def test_topic(self):
        from std_msgs.msg import String
        source = TopicSource('/test_topic', String)
        sink = TopicSink('/test_topic', String)
        with source, sink:
            test_strs = ['a', 'b', 'c']
            for s in test_strs:
                sink.put(s, rospy.Time.now())
            for msg, t in source:
                self.assertTrue(msg.data in test_strs)
                i = test_strs.index(msg.data)
                test_strs.pop(i)
                if not test_strs:
                    return

if __name__ == '__main__':
        import rosunit
        rosunit.unitrun(PKG, NAME, TestTopic)
