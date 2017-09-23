from styx_msgs.msg import TrafficLight
from keras.models import model_from_json
from scipy.misc import imresize
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self):
	# Load Keras model
	json_file = open('/home/student/CarND-Capstone/ros/src/tl_detector/light_classification/model.json', 'r')
	json_model = json_file.read()
	json_file.close()
	self.model = model_from_json(json_model)
	self.model.load_weights('/home/student/CarND-Capstone/ros/src/tl_detector/light_classification/model.h5')
	self.model._make_predict_function()
	self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Resize to trained image size
	image = imresize(image, (180,320,3))
	image = np.array(image)
    	image = image[None,:,:,:]
	with self.graph.as_default():
		prediction = self.model.predict(image)
	
	if np.argmax(prediction) == 1:
		return TrafficLight.RED
	else:
        	return TrafficLight.UNKNOWN
