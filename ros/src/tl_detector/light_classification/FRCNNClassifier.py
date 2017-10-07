import numpy as np
import tarfile
import tensorflow as tf
import urllib2
from matplotlib import pyplot as plt
import pathlib
from utils import visualization_utils as vis_util
# from utils import label_map_util
import time
import cv2
import os

import glob
from styx_msgs.msg import TrafficLight

MODEL_URL='http://storage.googleapis.com/download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz'
MODEL_NAME='./faster_rcnn_resnet101_coco_11_06_2017'
# MODEL_NAME='./bosch-1class-70k'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

class FRCNNClassifier(object):
  detection_graph = None
  session = None

  def __init__(self):
    # see if model exists, if not get it!
    if (not pathlib.Path(MODEL_NAME).is_dir()):
      print ('Downloading Faster RCNN from google')
      os.system('wget '+MODEL_URL)
      tar = tarfile.open(MODEL_NAME + '.tar.gz','r:gz')
      tar.extractall()
      tar.close()

    self.detection_graph = tf.Graph()
    with self.detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
      self.session = tf.Session(graph=self.detection_graph)

  def get_light_state(self, light, thresh=200):
    light = cv2.cvtColor(light, cv2.COLOR_BGR2HLS)
    h, v, s = cv2.split(light)

    state = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN]
    splith = int(light.shape[0] / len(state))
    state_out = np.zeros(3)
    max_state = -1
    max_state_val = 0
    for i in range(len(state)):
      state_out[i] = len(np.where(s[splith * i: min(splith * (i + 1), light.shape[0]), :] > thresh)[0])
      if (state_out[i] > max_state_val):
        max_state_val = state_out[i]
        max_state = i

    if (max_state >= 0):
      return state[max_state], state_out
    else:
      return 'Off', None

  def extract_traffic_light(self, image, boxes, scores, classes, class_light=10):
    scores = np.squeeze(scores)
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    image_to_ret = None
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
          print (
          'acc score: {} return x: {},{} y: {},{} shape: {}'.format(scores[i], startx, endx, starty, endy, image.shape))
          image_to_ret = image[starty: endy, startx:endx, :]
        else:
          print (
          'rej score: {} return x: {},{} y: {},{} shape: {}'.format(scores[i], startx, endx, starty, endy, image.shape))
    if (image_to_ret is None):
      print ('couldnt find light!')
    return image_to_ret

  def get_classification(self, image):
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
    start = time.time()
    # image = Image.open(image_path)
    image_np = np.array(image)
    # image = image.resize((1200,900))
    # the array based representation of the image will be used later in order to prepare the
    # result image with boxes and labels on it.
    # image_np = load_image_into_numpy_array(image)
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    # Actual detection.
    diff1 = time.time() - start
    (boxes, scores, classes, num) = self.session.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})
    diff2 = time.time() - start
    # Visualization of the results of a detection.
    # class == 10 for coco, 1 for ours
    light = self.extract_traffic_light(image_np, np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes),class_light=10)
    state = 'No Light'
    if (light is not None):
      state, values = self.get_light_state(light)
    print state
    cv2.putText(image_np, str(state), (230, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
    # print ct.get_palette(color_count=10)
    # print image.getcolors(5000)

    # plt.imshow(light)
    # plt.show()
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        {},
        use_normalized_coordinates=True,
        line_thickness=4,
        min_score_thresh=0.5)
    if (True):
      plt.imshow(image_np)
      plt.show()
    return state

if __name__ == '__main__':
    a = FRCNNClassifier()
    for file in sorted(glob.glob('/home/faraz/camera/*.png')):
      image = cv2.imread(file)
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      print(a.get_classification(image))

