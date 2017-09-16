#!/usr/bin/env python
import sys
import argparse
import rosbag
import os
from msg_extractor import ROSMsgExtractor

"""
extract_export: extracts ros messages from a ROSbag
"""

def main():
    """ Parse args, bootstrap the ROSMsgExtractor, and process all messages """

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

    extractor = ROSMsgExtractor(output_dir=output_dir)

    print("reading rosbag ", bag_file)
    bag = rosbag.Bag(bag_file, 'r')
    topicTypesMap = bag.get_type_and_topic_info().topics

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
                    extractor.print_msg(msgType, topic, msg, t)
                if args.display or output_dir:
                    extractor.handle_msg(msgType, topic, msg, t)

# ***** main loop *****
if __name__ == "__main__":
    main()
