from styx_msgs.msg import TrafficLight
from keras.models import model_from_json
from scipy.misc import imresize
import tensorflow as tf
import numpy as np
import os
class TLClassifier(object):
    def __init__(self):
	# Load Keras model
	dir_path = os.path.dirname(os.path.realpath(__file__))
	json_file = open(dir_path+'/model.json', 'r')
	json_model = json_file.read()
	json_file.close()
	self.model = model_from_json(json_model)
	self.model.load_weights(dir_path+'/model.h5')
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
	
	# Label indices: Green: 0, Red: 1, Yellow: 2, None: 3
	pred = np.argmax(prediction)
	if pred == 1:
		return TrafficLight.RED
	elif pred == 0:
		return TrafficLight.GREEN
	elif pred == 2:
		return TrafficLight.YELLOW
	else:
        	return TrafficLight.UNKNOWN
