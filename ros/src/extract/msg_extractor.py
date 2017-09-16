import matplotlib.image as mpimg
import cv2
import os
from cv_bridge import CvBridge

class ROSMsgExtractor:

    def __init__(self,
                 window_max_width=875,
                 output_dir=None):
        self.windows = {}
        self.bridge = CvBridge()
        self.window_max_width = window_max_width
        self.output_dir = output_dir

        if output_dir is not None:
            if not (os.path.isdir(self.output_dir + '/camera/')):
                os.makedirs(self.output_dir + '/camera/')

    @staticmethod
    def save_image(output_dir, name, count, image):
        """Save image to RGB format"""

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imwrite('{}/{}_{}.png'.format(output_dir, name, count), image)

    @staticmethod
    def print_msg(msgType, topic, msg, time, startsec):
        """Print ROS messages by message type"""

        t = time.to_sec()
        since_start = 0

        if 'sensor_msgs' in msgType or 'nav_msgs' in msgType:
            since_start = msg.header.stamp.to_sec() - startsec

        if msgType == 'sensor_msgs/PointCloud2':
            print(topic, msg.header.seq, since_start, 'nPoints=', msg.width * msg.height, t)

        elif msgType == 'sensor_msgs/NavSatFix':
            print(topic, msg.header.seq, since_start, msg.latitude, msg.longitude, msg.altitude, t)

        elif msgType == 'nav_msgs/Odometry':

            position = msg.pose.pose.position
            print(topic, msg.header.seq, since_start, position.x, position.y, position.z, t)

        elif msgType == 'sensor_msgs/Range':

            print(topic, msg.header.seq, since_start, msg.radiation_type, msg.field_of_view, msg.min_range, msg.max_range,
                  msg.range, t)

        elif msgType == 'sensor_msgs/Image':

            print(topic, msg.header.seq, msg.width, msg.height, since_start, t)

        elif msgType == 'sensor_msgs/CameraInfo':

            print(topic, msg.header.seq, since_start, msg.width, msg.height, msg.distortion_model, t)

        else:
            pass
            # print(topic, msg.header.seq, t-msg.header.stamp, msg, t)

    @staticmethod
    def save_images(output_dir, count, images):
        """Save images using matplotlib"""

        for k, img in images.iteritems():
            mpimg.imsave('{}/{}_{}.png'.format(output_dir, count, k), images[k], origin='upper')

    def handle_msg(self, msg_type, topic, msg, timestamp):
        """Generic handler ROS messages.

        For images, extract the image pixels from the ROS message and then save using the timestamp as the name.

        """

        window = []
        img = []

        if msg_type in ['sensor_msgs/Image']:

            cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            name = 'image'
            if 'center' in topic:
                name = 'center'
            elif 'left' in topic:
                name = 'left'
            elif 'right' in topic:
                name = 'right'

            if self.output_dir is not None:
                self.save_image(self.output_dir + '/camera/', name, timestamp, cv_img)
