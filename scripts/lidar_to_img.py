#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys
import time
import threading
import multiprocessing

# External modules
import cv2
import numpy as np
import matplotlib.cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# # ROS modules
PKG = "hyper_drive"
import roslib

roslib.load_manifest(PKG)
import rospy
import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

"""
Callback function to publish project image and run calibration

Inputs:

Outputs: None
"""


def callback(image, camera_info, velodyne, image_pub=None):
    # TF_BUFFER = tf2_ros.Buffer()
    # TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)
    CV_BRIDGE = CvBridge()
    CAMERA_MODEL = image_geometry.PinholeCameraModel()
    CAMERA_MODEL.fromCameraInfo(camera_info)
    CAMERA_MODEL.full_K = np.array(
        [
            [1531.612907, 0.000000, 423.401732],
            [0.000000, 1530.678200, 331.947453],
            [0.000000, 0.000000, 1.000000],
        ]
    )
    CAMERA_MODEL.D = np.array([-0.150858, 0.367536, 0.000146, 0.000102, 0.000000])

    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Transform the point cloud
    try:
        transform = TF_BUFFER.lookup_transform("world", "velodyne", rospy.Time())
        velodyne = do_transform_cloud(velodyne, transform)
    except tf2_ros.LookupException:
        pass

    # Extract points from message
    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne)
    points3D = np.asarray(points3D.tolist())

    # Filter points in front of camera
    inrange = np.where(
        (points3D[:, 2] > 0)
        & (points3D[:, 2] < 10)
        & (np.abs(points3D[:, 0]) < 10)
        & (np.abs(points3D[:, 1]) < 10)
    )
    max_intensity = np.max(points3D[:, -1])
    points3D = points3D[inrange[0]]

    # Color map for the points
    cmap = matplotlib.cm.get_cmap("jet")
    colors = cmap(points3D[:, -1] / max_intensity) * 255

    # Project to 2D and filter points within image boundaries
    points2D = [CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3]]
    points2D = np.asarray(points2D)
    inrange = np.where(
        (points2D[:, 0] >= 0)
        & (points2D[:, 1] >= 0)
        & (points2D[:, 0] < img.shape[1])
        & (points2D[:, 1] < img.shape[0])
    )
    points2D = points2D[inrange[0]].round().astype("int")

    # Draw the projected 2D points
    for i in range(len(points2D)):
        cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)

    # Publish the projected points image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr(e)


"""
The main ROS node which handles the topics

Inputs:
    camera_info - [str] - ROS sensor camera info topic
    image_color - [str] - ROS sensor image topic
    velodyne - [str] - ROS velodyne PCL2 topic
    camera_lidar - [str] - ROS projected points image topic

Outputs: None
"""


def listener(camera_info, image_color, velodyne_points, camera_lidar=None):
    # Start node
    rospy.init_node("calibrate_camera_lidar", anonymous=True)
    rospy.loginfo("*****************************")
    rospy.loginfo("Current PID: [%d]" % os.getpid())
    rospy.loginfo("CameraInfo topic: %s" % camera_info)
    rospy.loginfo("Image topic: %s" % image_color)
    rospy.loginfo("PointCloud2 topic: %s" % velodyne_points)
    rospy.loginfo("Output topic: %s" % camera_lidar)
    rospy.loginfo("*****************************")

    global TF_BUFFER
    global TF_LISTENER

    TF_BUFFER = tf2_ros.Buffer()
    TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)

    # Subscribe to topics
    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    image_sub = message_filters.Subscriber(image_color, Image)
    velodyne_sub = message_filters.Subscriber(velodyne_points, PointCloud2)

    # Publish output topic
    image_pub = None
    if camera_lidar:
        image_pub = rospy.Publisher(camera_lidar, Image, queue_size=5)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, info_sub, velodyne_sub], queue_size=100, slop=0.1
    )
    ats.registerCallback(callback, image_pub)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":

    # camera_info = rospy.get_param("camera_info_topic")
    # image_color = rospy.get_param("image_color_topic")
    # velodyne_points = rospy.get_param("velodyne_points_topic")
    # camera_lidar = rospy.get_param("camera_lidar_topic")
    camera_info = "/camera/camera_info"
    image_color = "/camera/image_raw"
    velodyne_points = "/velodyne_points"
    camera_lidar = "/camera/camera_lidar"

    # Start subscriber
    listener(camera_info, image_color, velodyne_points, camera_lidar)
