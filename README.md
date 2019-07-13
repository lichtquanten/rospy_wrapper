# rospy_wrapper

A package that allows you to interact with ROS bags, live ROS topics, and other data inputs and outputs in the same way.

## Usage
Get messages from a ROS subscription
```python
from rospywrapper import ROSLiveSource
from sensor_msgs.msg import Image

with ROSLiveSource(
    topic='/camera/image',
    data_class=Image) as source:
     
    for msg, t in source:
        print msg['data']
```
Get messages from a rosbag
```python
from rospywrapper import ROSBagSource

with ROSBagSource(
    topic='/audio',
    filename='input.bag') as source:
     
    for msg, t in source:
        print msg['data']
```
Create a custom source
```python
from rospywrapper import Source
import numpy as np
import cv2

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

  def __exit__(self):
      self.video.release()

  def __iter__(self):
      return self

  def __next__(self):
      while self.video.isOpened():
          ret, frame = self.video.read()
          yield frame, self.time
          self.time += self.step
```
Use the custom source
```python
with VideoFileSource('vtest.avi') as source:
  for frame, t in source:
      cv2.imshow('frame', frame)
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
```
