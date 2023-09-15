#!/usr/bin/env python3
"""
本文件启动一个同名节点替代PL-VINS中的feature_tracker_node
功能是监听图像信息并使用导入的自定义点特征提取模块来进行detecting&tracking
"""
import cv2
import os
import sys
import copy
import rospy
import torch
import yaml
import json
from time import time

import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import String
from time import time

from utils.parameter import read_image
from utils.camera_model import CameraModel
# from feature_tracker_module import PLFeatureTracker
from feature_tracker_extract import PLFeatureTrackerExtract

from utils_pl.my_pl_model import create_plextract_instance, create_pointmatch_instance, create_linematch_instance
from feature_tracker.msg import ExtractedFeature

init_pub = False
count_frame = 0


def img_callback(img_msg, params_dict):
    feature_tracker_extract = params_dict["feature_tracker_extract"]
    height = params_dict["H"]
    width = params_dict["W"]
    global init_pub
    global count_frame

    if not init_pub:
        init_pub = True
    else:
        init_pub = False

        bridge = CvBridge()
        conver_img = bridge.imgmsg_to_cv2(img_msg, "mono8")

        # scale = 2
        feature_tracker_extract.readImage(conver_img)

        if True:
            extracted_msg=ExtractedFeature()

            start_time=time()
            # extracted_msg.PointID=feature_tracker_extract.forwframe_['PointID']
            # extracted_msg.keyPoint=list(feature_tracker_extract.forwframe_['keyPoint'].flatten())
            # extracted_msg.keyPoint_shape=feature_tracker_extract.forwframe_['keyPoint'].shape
            # extracted_msg.vecline=list(feature_tracker_extract.forwframe_['vecline'].flatten())
            # extracted_msg.vecline_shape=feature_tracker_extract.forwframe_['vecline'].shape
            # extracted_msg.lineID=feature_tracker_extract.forwframe_['lineID']
            # extracted_msg.linedescriptor=list(feature_tracker_extract.forwframe_['linedescriptor'].flatten())
            # extracted_msg.linedescriptor_shape=feature_tracker_extract.forwframe_['linedescriptor'].shape
            # extracted_msg.pointdescriptor=list(feature_tracker_extract.forwframe_['pointdescriptor'].flatten())
            # extracted_msg.pointdescriptor_shape=feature_tracker_extract.forwframe_['pointdescriptor'].shape
            # extracted_msg.valid_points=list(feature_tracker_extract.forwframe_['valid_points'].flatten())
            # extracted_msg.valid_points_shape=feature_tracker_extract.forwframe_['valid_points'].shape
            # extracted_msg.image=bridge.cv2_to_imgmsg(conver_img,"mono8")
            # pub_feature_extract.publish(extracted_msg) 


            extracted_msg.PointID=feature_tracker_extract.forwframe_['PointID']
            extracted_msg.keyPoint_shape=feature_tracker_extract.forwframe_['keyPoint'].shape
            extracted_msg.keyPoint=feature_tracker_extract.forwframe_['keyPoint'].ravel().tolist()
            extracted_msg.vecline_shape=feature_tracker_extract.forwframe_['vecline'].shape
            extracted_msg.vecline=feature_tracker_extract.forwframe_['vecline'].ravel().tolist()
            extracted_msg.lineID=feature_tracker_extract.forwframe_['lineID']
            extracted_msg.linedescriptor_shape=feature_tracker_extract.forwframe_['linedescriptor'].shape
            extracted_msg.linedescriptor=feature_tracker_extract.forwframe_['linedescriptor'].ravel().tolist()
            extracted_msg.pointdescriptor_shape=feature_tracker_extract.forwframe_['pointdescriptor'].shape
            extracted_msg.pointdescriptor=feature_tracker_extract.forwframe_['pointdescriptor'].ravel().tolist()
            extracted_msg.valid_points_shape=feature_tracker_extract.forwframe_['valid_points'].shape
            extracted_msg.valid_points=feature_tracker_extract.forwframe_['valid_points'].ravel().tolist()
            extracted_msg.image=bridge.cv2_to_imgmsg(conver_img,"mono8")
            pub_feature_extract.publish(extracted_msg) 



            
            end_time=time()
            print("### serialize_msg_time: ",end_time-start_time)    




if __name__ == '__main__':
    rospy.init_node('feature_tracker_extract', anonymous=False)
    yamlPath = rospy.get_param(
        "~config_path", "/home/xm/catkin_ws/src/sp-sold2-vins/config/feature_tracker/sp-sold2_config.yaml")

    with open(yamlPath, 'rb') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)
        pl_params = params["pl_feature_cfg"]
        point_params = params["point_feature_cfg"]
        line_params = params["line_feature_cfg"]

        camera_params = params["camera_cfg"]

    my_plextract_model = create_plextract_instance(pl_params)  # 建立点线提取模型
    # my_pointmatch_model = create_pointmatch_instance(
    #     point_params)  # 建立自定义点特征匹配模型
    # my_linematch_model = create_linematch_instance(line_params)  # 建立自定义线特征匹配模型

    camera_model = CameraModel(camera_params)
    CameraIntrinsicParam = camera_model.generateCameraModel()   # 建立相机模型
    feature_tracker_extract = PLFeatureTrackerExtract(my_plextract_model, CameraIntrinsicParam,
                                       num_samples=line_params["num_samples"], min_point_cnt=point_params["min_cnt"], min_line_cnt=line_params["min_cnt"])  # 利用点特征模型和相机模型生成点特征处理器

    image_topic = params["image_topic"]
    rospy.loginfo(
        "PLfeature Tracker initialization completed, waiting for img from topic: %s", image_topic)


    pub_feature_extract=rospy.Publisher("feature_extract_forw",ExtractedFeature,queue_size=1000)

    sub_img = rospy.Subscriber(image_topic, Image, img_callback,
                               {"feature_tracker_extract": feature_tracker_extract,
                                   "H": pl_params["H"], "W": pl_params["W"]},
                               queue_size=100)  # 监听图像，提取和追踪点线特征并发布




    # pub_point_img = rospy.Publisher("~feature", PointCloud, queue_size=1000)
    # pub_point_match = rospy.Publisher("~feature_img", Image, queue_size=1000)
    # pub_line_img = rospy.Publisher("~linefeature", PointCloud, queue_size=1000)
    # pub_line_match = rospy.Publisher(
    #     "~linefeature_img", Image, queue_size=1000)

    rospy.spin()