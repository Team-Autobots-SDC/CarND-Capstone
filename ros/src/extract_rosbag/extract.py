#!/usr/bin/env python
import sys
sys.path.append('../')
import argparse
import numpy as np
import rosbag
import os
import matplotlib.image as mpimg
import csv
import cv2
from cv_bridge import CvBridge

class ROSBagExtractor:

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

        self.mapx, self.mapy, self.new_size = None, None, None

    @staticmethod
    def save_image(output_dir, name, count, image, mapx = None, mapy = None, new_size = None):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if mapx is not None:
            image = remap(image,mapx,mapy,new_size)
        cv2.imwrite('{}/{}_{}.png'.format(output_dir, name, count), image)

    @staticmethod
    def print_msg(msgType, topic, msg, time, startsec):
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
        for k, img in images.iteritems():
            mpimg.imsave('{}/{}_{}.png'.format(output_dir, count, k), images[k], origin='upper')

    def handle_msg(self, msg_type, topic, msg, timestamp, result):

        window = []
        img = []

        if '/radar/tracks' in topic:
            tracks = radar_tracks.parse_msg(msg, timestamp)
            if len(tracks) > 0:
                self.radar_tracks += tracks

        elif msg_type in ['sensor_msgs/Image']:

            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

            name = 'image'
            if 'center' in topic:
                name = 'center'
            elif 'left' in topic:
                name = 'left'
            elif 'right' in topic:
                name = 'right'

            if self.output_dir is not None:
                self.save_image(self.output_dir + '/camera/', name, timestamp, cv_img, self.mapx, self.mapy, self.new_size)

def write_timestamps_to_csv(timestamps, output_file):

    csv_file = open(output_file, 'w')
    writer = csv.DictWriter(csv_file, ['timestamp'])

    writer.writeheader()

    for ts in timestamps:
        writer.writerow({'timestamp': ts})


def main():

    appTitle = "Udacity Team-Autobots: ROSbag viewer"
    parser = argparse.ArgumentParser(description=appTitle)
    parser.add_argument('bag_file', type=str, help='ROS Bag name')
    parser.add_argument('--skip', type=float, default="0", help='skip seconds')
    parser.add_argument('--length', type=float, default=None, help='length seconds')
    parser.add_argument('--topics', type=str, default=None, help='Topic list to display')
    parser.add_argument('--outdir', type=str, default=None, help='Output directory for images')
    parser.add_argument('--quiet', dest='quiet', action='store_true', help='Quiet mode')
    parser.set_defaults(quiet=False, display=False)

    args, unknown = parser.parse_known_args()

    bag_file = args.bag_file
    output_dir = args.outdir

    if not os.path.isfile(bag_file):
        print('bag_file ' + bag_file + ' does not exist')
        sys.exit()

    if output_dir is not None and not(os.path.isdir(output_dir)):
        print('output_dir ' + output_dir + ' does not exist')
        sys.exit()

    skip = args.skip
    length = args.length
    startsec = 0
    last_topic_time = {}
    maximum_gap_topic = {}
    topics_list = args.topics.split(',') if args.topics else None

    extractor = ROSBagExtractor(output_dir=output_dir)

    print("reading rosbag ", bag_file)
    bag = rosbag.Bag(bag_file, 'r')
    topicTypesMap = bag.get_type_and_topic_info().topics

    result = {'intensity': {}, 'distance': {}, 'height': {}}
    for topic, msg, t in bag.read_messages(topics=topics_list):
        msgType = topicTypesMap[topic].msg_type
        if startsec == 0:
            startsec = t.to_sec()
            if skip < 24 * 60 * 60:
                skipping = t.to_sec() + skip
                print("skipping ", skip, " seconds from ", startsec, " to ", skipping, " ...")
            else:
                skipping = skip
                print("skipping to ", skip, " from ", startsec, " ...")
        else:
            if t.to_sec() > skipping:

                if length is not None and t.to_sec() > skipping + length:
                    break

                if last_topic_time.get(topic) != None:
                    gap = t.to_sec() - last_topic_time[topic]
                    if maximum_gap_topic.get(topic) == None or gap > maximum_gap_topic[topic]:
                        maximum_gap_topic[topic] = gap

                last_topic_time[topic] = t.to_sec()

                if not args.quiet:
                    extractor.print_msg(msgType, topic, msg, t, startsec)
                if args.display or output_dir:
                    extractor.handle_msg(msgType, topic, msg, t, result)

# ***** main loop *****
if __name__ == "__main__":
    main()
