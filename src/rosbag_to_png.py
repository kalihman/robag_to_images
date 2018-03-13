#!/usr/bin/python

PKG = 'rosbag_to_images'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge, CvBridgeError
import os
import argparse, sys

# Parse optional arguments to use
parser = argparse.ArgumentParser()
parser.add_argument('--topic_name', help='Image topic to extract')
parser.add_argument('--save_dir', help='Absolute path to directory to save images')
parser.add_argument('--save_name', help='Name pattern of the image files to save (default: image_*)', default='image_')
parser.add_argument('--bagfile', help='Absolute path to the bagfile')
parser.add_argument('--format', help='Image format to save file, should be supported by opencv((default: png)', default='png')
args = parser.parse_args()

class RosbagToImages():
    def __init__(self):
        # Get parameters when starting node from a launch file.
        if len(args) < 5:
            topic_name = rospy.get_param('topic_name')
            save_name = rospy.get_param('save_name')
            image_format = rospy.get_param('format')
            if os.path.exists(rospy.get_param('save_dir'):
                save_dir = rospy.get_param('save_dir')
            else:
                sys.exit("The save directory does not exists")
            if os.path.exists(rospy.get_param('bagfile'):
                bagfile = rospy.get_param('bagfile')
            else:
                sys.exit("Bagfile does not exists")
        
        # Get parameters as arguments 
        # 'rosrun rosbag_to_images rosbag_to_images.py --topic_name <topic> --save_dir <save_dir> --save_name <save_name> --bagfile <bagfile> --format <format>'
        else:
            topic_name = args.topic_name
            save_name = args.save_name
            image_format = args.format
            if os.path.exists(args.save_dir):
                save_dir = args.save_dir
            else:
                sys.exit("The save directory does not exists")
            if os.path.exists(args.bagfile):
                bagfile = os.path.abspath(args.bagfile)
            else:
                sys.exit("Bagfile does not exists")
        
        # Log input information
        rospy.loginfo("Topic: %s", topic_name)
        rospy.loginfo("Bagfile: %s", bagfile)
        rospy.loginfo("Save name pattern: %s", save_name)
        rospy.loginfo("Save directory: %s", save_dir)
        rospy.loginfo("Image format: %s", image_format)

        # Convert ROS Image to OpenCV Image
        self.bridge = CvBridge()

        # Open bag file, search topic, extract images, and save them to given directory
        with rosbag.Bag(str(bagfile), 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == topic_name:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError, e:
                        print e
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    image_name = str(save_dir) + save_name + timestr + image_format
                    cv.SaveImage(image_name, cv.fromarray(cv_image))

# Main function.    
if __name__ == '__main__':
    # Initialize the node
    rospy.init_node(PKG)

    # Create RosbagToImages object, which does all the job when initiated
    # TODO: make separate functions to do the extraction and saving tasks
    try:
        rosbag_to_images = RosbagToImages()
    except rospy.ROSInterruptException: 
        pass
