# rospy_wrapper

A package that allows easy interchanging of ROS topics, rosbags, and other sources of and sinks for data.

Note: To use TopicSource or TopicSink, there must have been a previous call to rospy.init_node().

## Usage
Instantiate a topic source
```python
from rospywrapper import TopicSource
from sensor_msgs.msg import Image

source = TopicSource(
    topic='/camera/image',
    data_class=Image)
```
Instantiate a rosbag source
```python
from rospywrapper import BagSource

source = BagSource(
    topic='/camera/image',
    pathname='input.bag')
```
Use a source to show images
```python
import cv2
import numpy as np

with source:
  for msg, t in source:
    data = np.fromstring(msg['data'], np.uint8)
    img = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow(img)
    cv2.waitKey(2)
```
Instantiate and use a topic sink
```python
from rospywrapper import TopicSink

sink = TopicSink(
    topic='/camera/gray',
    data_class=Image)
with sink:
    # Use sink
```
Instantiate and use a bag sink
```python
import rosbag
from rospywrapper import BagSink

bag = rosbag.Bag('out.bag', 'w')
sink = BagSink(
    bag=bag,
    topic='/camera/image',
    data_class=Image)
with bag:
    with sink:
        # Use sink
```
Write from source to sink
```python
import conditional

if use_bag:
    source = BagSource('in.bag', '/camera/image')
    bag = rosbag.Bag('out.bag', 'w')
    sink = BagSink(bag, '/camera/gray', Image)
else:
    source = TopicSource('/camera/image', Image)
    bag = None
    sink = TopicSink('/camera/gray', Image)

with conditional(bag is not None, bag):
    with source, sink:
      for msg, t in source:
        data = np.fromstring(msg['data'], np.uint8)
        img = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sink.put(gray, t)
```
Create a custom source
```python
import cv2
import rospy

from rospywrapper import Source

class VideoFileSource(Source):
  def __init__(self, filename, start_time=0):
      self.filename = filename
      self.time = start_time
      self.step = None

  def __enter__(self):
      self.video = cv2.VideoCapture(self.filename)
      fps = self.video.get(cv2.CAP_PROP_FPS)
      self.step = 1./fps
      return self

  def __exit__(self, *exc):
      self.video.release()

  def __iter__(self):
      return self

  def next(self):
      if not self.video.isOpened():
          raise StopIteration
      _, frame = self.video.read()
      t = self.time
      self.time += self.step
      return frame, rospy.Time.from_sec(t)
```
Create a custom sink
```python
from rospywrapper import Sink
import csv

class CSVSink(Sink):
    def __init__(self, filename):
        self.filename = filename
        self._file = None
        self._writer = None

    def put(self, topic, data_class, data, t):
        row = [t] + [data[key] for key in data]
        self._writer.writerow(row)

    def __enter__(self):
        self._file = open(self.filename, 'w')
        self._writer = csv.writer(self.file)
        return self

    def __exit__(self, *exc):
        self._file.close()
```
