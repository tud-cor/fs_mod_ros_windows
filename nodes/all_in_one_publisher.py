#!/usr/bin/env python
# Copyright (c) 2021, TU Delft
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# author: Ting-Chia Chiang
# author: G.A. vd. Hoorn

# named pipe modules
import time
import math
import sys
import win32pipe, win32file, pywintypes
import os
import os.path
import struct

import subprocess as sp

# common ROS modules
import roslib
import rospy
from rospy_message_converter import json_message_converter

# module for Sim time 
from rosgraph_msgs.msg import Clock


# modules for Odom
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform, PoseStamped
import tf2_ros


# modules for laser scan
from sensor_msgs.msg import LaserScan
from tf_conversions import transformations


# modules for imu
from sensor_msgs.msg import Imu

# module for tf
from tf2_msgs.msg import TFMessage

class ROSMessagePublisher:
    
    
    # some class variables shared across all instances of the class

    # sim time message initialization 
    pub_sim = rospy.Publisher('clock', Clock, queue_size=10)  
 

    # odom messages initialization
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    transformStamped = t = TransformStamped()
    # instantiate an object of transformbroadcaster
    odom_broadcaster = tf2_ros.TransformBroadcaster()
    base_footprint = Odometry()


    # laser scan messages initialization
    pub_laser = rospy.Publisher('scan', LaserScan, queue_size=10)  
    t_laser = TransformStamped()
    # instantiate an object of transformbroadcaster
    laser_scan_broadcaster = tf2_ros.TransformBroadcaster()

    # instantiate object of imu publisher
    pub_imu = rospy.Publisher('imu', Imu, queue_size=10)  

    # instantiate object of tf publisher
    pub_tf = rospy.Publisher('tf', TFMessage, queue_size=10)  


    def laser_scan_pub(self, scan):

        for count, value in enumerate(scan.ranges):
            if float(value) == 1000:
                scan.ranges[count] = float('Inf')
                
        self.pub_laser.publish(scan)

    @staticmethod
    def create_pipe(pipe_name):
        pipe_path = "\\\\.\pipe\\" + pipe_name

        security_attributes = pywintypes.SECURITY_ATTRIBUTES()
        '''the maximum number of instances that can be created for this pipe: unlimited 
        so that we are allowed to create a new instance of an existing named pipe for other messages in other files'''
        PIPE_UNLIMITED_INSTANCES = 255
        #  the second parameter "pDacl", if it is NULL, a NULL DACL is assigned to the security descriptor, which allows all access to the object
        security_attributes.SetSecurityDescriptorDacl(1, None, 1)
        pipe = win32pipe.CreateNamedPipe(
        pipe_path,
        win32pipe.PIPE_ACCESS_DUPLEX,
        win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_READMODE_MESSAGE | win32pipe.PIPE_WAIT,
        PIPE_UNLIMITED_INSTANCES, 65536, 65536,
        0,
        security_attributes)

        return pipe




if __name__ == '__main__':

    try:
        named_pipe_path = os.path.join(os.environ['USERPROFILE'], "Documents/My Games/FarmingSimulator2019/mods/modROS/ROS_messages")
        # check if a symbolic link to a named pipe has been created
        if not (os.path.islink(named_pipe_path)):
            print("symbolic link has not yet been created, creating symbolic link now (make sure cmd shell is elevated)")
            # source file path
            src = "\\\\.\pipe\ROS_messages"
            # destination file path
            dst = named_pipe_path
            try:
                os.symlink(src, dst)
            except OSError as e:
                sys.exit(f"Unable to symlink the named pipe: {e}")
            print("symbolic link to named pipe has been created successfully")
        else:
            print("symbolic link has already been created")
        
        # wait a bit to ensure that symbolic links have been created
        time.sleep(1)
        
        object_class = ROSMessagePublisher()


        rospy.init_node('ros_publisher', anonymous=True)
        pipe = object_class.create_pipe("ROS_messages")
        print("waiting for client from FarmSim19")
        win32pipe.ConnectNamedPipe(pipe, None)
        print("got client from game!!")
        
     

        while not rospy.is_shutdown():
           
            resp = win32file.ReadFile(pipe, 64*1024)
            #  convert json data from lua to ros message (data read from the pipe is in bytes: need to convert to string)

            msg_list = resp[1].decode("utf-8").split("\n")
            if msg_list[0] == "rosgraph_msgs/Clock":
                sim_time_msg = json_message_converter.convert_json_to_ros_message('rosgraph_msgs/Clock', msg_list[1])
                object_class.pub_sim.publish(sim_time_msg)
                
            elif msg_list[0] == "nav_msgs/Odometry":
                odom_msg = json_message_converter.convert_json_to_ros_message('nav_msgs/Odometry', msg_list[1])
                object_class.pub_odom.publish(odom_msg)
            
            elif msg_list[0] == "sensor_msgs/LaserScan":
                scan_msg = json_message_converter.convert_json_to_ros_message('sensor_msgs/LaserScan', msg_list[1])
                object_class.laser_scan_pub(scan_msg)
                
            elif msg_list[0] == "sensor_msgs/Imu":
                imu_msg = json_message_converter.convert_json_to_ros_message('sensor_msgs/Imu', msg_list[1])
                object_class.pub_imu.publish(imu_msg)
       
            elif msg_list[0] == "tf2_msgs/TFMessage":
                tf_msg = json_message_converter.convert_json_to_ros_message('tf2_msgs/TFMessage', msg_list[1])
                object_class.pub_tf.publish(tf_msg)
           

    except rospy.ROSInterruptException:
        pass
