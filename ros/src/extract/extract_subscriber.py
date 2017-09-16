#!/usr/bin/env python

"""
extract_subscriber: captures ros messages on specified topics at a given rate
"""

import sys
import argparse
import os
import rospy
import time
from sensor_msgs.msg import Image
from msg_extractor import ROSMsgExtractor

def main():
    rospy.init_node('extract_subscriber')

    appTitle = "Udacity Team-Autobots: ROS Message Extractor"
    parser = argparse.ArgumentParser(description=appTitle)
    parser.add_argument('topics', type=str, default=None, help='Topic list to display')
    parser.add_argument('--rate', type=float, default="10", help='skip seconds')
    parser.add_argument('--skip', type=float, default="0", help='skip seconds')
    parser.add_argument('--length', type=float, default=None, help='length seconds')
    parser.add_argument('--outdir', type=str, default=None, help='Output directory for images')
    parser.add_argument('--quiet', dest='quiet', action='store_true', help='Quiet mode')
    parser.set_defaults(quiet=False, display=False)

    args, unknown = parser.parse_known_args()

    quiet = args.quiet
    timeout = 1.0 / args.rate
    output_dir = args.outdir
    extractor = ROSMsgExtractor(output_dir=output_dir)
    startsec = rospy.get_rostime().to_sec()
    last_t = [startsec]

    def listen(msg):

        msgType = msg._connection_header['type']
        topic = msg._connection_header['topic']
        t = rospy.get_rostime()

        if t.to_sec() - last_t[0] > timeout:

            if msgType == 'sensor_msgs/Image':
                # assumes the camera uses rgb8 images
                msg.encoding = "rgb8"

            if not quiet:
                extractor.print_msg(msgType, topic, msg, t, startsec)
            if output_dir:
                extractor.handle_msg(msgType, topic, msg, t)

            last_t[0] = t.to_sec()

    for topic in args.topics.split(','):
        if 'image' in topic:
            print('Subscribing to {}'.format(topic))
            rospy.Subscriber(topic, Image, listen)

    rospy.spin()

if __name__ == '__main__':
    main()
