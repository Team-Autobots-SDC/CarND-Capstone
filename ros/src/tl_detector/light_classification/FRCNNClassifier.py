import numpy as np
import tensorflow as tf
import time
import cv2
import os
import rospy
import sys

import glob
import classify_light
from styx_msgs.msg import TrafficLight

MODEL_NAME='faster_rcnn_resnet101_coco_11_06_2017'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

class FRCNNClassifier(object):
  detection_graph = None
  session = None

  def __init__(self, path='./'):
    # see if model exists, if not get it!
    if os.path.isdir(path + './' + MODEL_NAME):
      rospy.loginfo('Loading Faster RCNN model with COCO')
    else:
      rospy.logerr('Missing Faster RCNN model with COCO. Run `download_rcnn_model.sh` first.')

    print('Loading tf graph...')
    self.detection_graph = tf.Graph()
    with self.detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(path + './' + PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
      self.session = tf.Session(graph=self.detection_graph)
    print('Loaded.')

  def extract_traffic_light(self, image, boxes, scores, classes, class_light=10):
    scores = np.squeeze(scores)
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    bounding_box = None
    highest_score = 0
    for i in range(boxes.shape[0]):
      if scores is None or scores[i] > 0.5 and classes[i] == class_light:

        if (scores[i] > highest_score):
          highest_score = scores[i]
          box = tuple(boxes[i].tolist())
          ymin, xmin, ymax, xmax = box
          startx = int(xmin * image.shape[1])
          endx = int(xmax * image.shape[1])
          starty = int(ymin * image.shape[0])
          endy = int(ymax * image.shape[0])
          # print (
          # 'acc score: {} return x: {},{} y: {},{} shape: {}'.format(scores[i], startx, endx, starty, endy, image.shape))
          bounding_box = [(startx, starty), (endx, starty), (startx, endy), (endx, endy)]

    if (bounding_box is None):
      rospy.loginfo('couldnt find light!')
    return bounding_box

  def extract_bounding_box(self, image_msg, light_msg):
    image_np = image_msg.data
    bounding_box, _, _, _, _ = self.extract_bounding_box_impl(image_np)
    return bounding_box

  def extract_bounding_box_impl(self, image_np):
    # Definite input and output Tensors for detection_graph
    with self.detection_graph.as_default():
      image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.https://console.cloud.google.com/logs/viewer?resource=ml_job%2Fjob_id%2Ffaraz_object_detection_1507318078&project=trainbosch
      detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
      detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    # Actual detection.
    (boxes, scores, classes, num) = self.session.run(
      [detection_boxes, detection_scores, detection_classes, num_detections],
      feed_dict={image_tensor: image_np_expanded})
    # Visualization of the results of a detection.
    # class == 10 for coco, 1 for ours
    light = self.extract_traffic_light(image_np, np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes),
                                      class_light=10)
    return light, boxes, scores, classes, num

  def get_classification(self, image_np, debug=False):
    start = time.time()
    light, boxes, scores, classes, num = self.extract_bounding_box_impl(image_np)
    state = 'No Light'
    if light is not None:
      state = classify_light.classify_light_with_bounding_box(light, image_np)
    diff = time.time() - start

    if (debug):
      from matplotlib import pyplot as plt
      from utils import visualization_utils as vis_util

      cv2.putText(image_np, str(state), (230, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
      vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          {},
          use_normalized_coordinates=True,
          line_thickness=4,
          min_score_thresh=0.5)
      print('Time taken: {}'.format(diff))
      plt.imshow(image_np)
      plt.show()

    return state

if __name__ == '__main__':
    a = FRCNNClassifier()
    for file in sorted(glob.glob(sys.argv[1])):
      image = cv2.imread(file)
      image = np.array(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
      print(a.get_classification(image, False))

