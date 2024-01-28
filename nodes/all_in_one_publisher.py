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

# common ROS modules
import rospy
from rospy_message_converter import json_message_converter

# module for Sim time 
from rosgraph_msgs.msg import Clock

# modules for Odom
from nav_msgs.msg import Odometry

# modules for laser scan
from sensor_msgs.msg import LaserScan

# modules for imu
from sensor_msgs.msg import Imu

# module for tf
from tf2_msgs.msg import TFMessage



class ROSMessagePublisher:
    def __init__(self):
        # instantiate an empty dictionary
        # it's used for storing an unique topic_name and publisher as key-value pairs in dict as class variable so that the publisher won't be created repeatedly
        # NOTE: topic_name = <namespace>/<topic_name> so it's always unique
        self.pub_dict = {}


    # store publisher object as value in a dict given topic name and message class
    def create_publisher(self, topic_name, ms_class):
        self.pub_dict[topic_name] = rospy.Publisher(topic_name, ms_class, queue_size=10)

def create_pipe(pipe_name):
    pipe_path = f"\\\\.\\pipe\\{pipe_name}"

    security_attributes = pywintypes.SECURITY_ATTRIBUTES()

    out_buf_sz = 64*1024
    in_buf_sz = 64*1024
    def_timeout = 0

    #  the second parameter "pDacl", if it is NULL, a NULL DACL is assigned to the security descriptor, which allows all access to the object
    security_attributes.SetSecurityDescriptorDacl(1, None, 1)
    pipe = win32pipe.CreateNamedPipe(
        pipe_path,
        win32pipe.PIPE_ACCESS_DUPLEX,
        win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_READMODE_MESSAGE | win32pipe.PIPE_WAIT,
        win32pipe.PIPE_UNLIMITED_INSTANCES, out_buf_sz, in_buf_sz,
        def_timeout,
        security_attributes
    )

    return pipe


if __name__ == '__main__':

    try:
        object_class = ROSMessagePublisher()

        rospy.init_node('ros_publisher', anonymous=True)
        pipe = create_pipe("FS19_modROS_pipe")
        print("waiting for client from FarmSim19")
        win32pipe.ConnectNamedPipe(pipe, None)
        print("got client from game!!")

        while not rospy.is_shutdown():
            read_sz = 64*1024
            resp = win32file.ReadFile(pipe, read_sz)

            #  convert json data from lua to ros message (data read from the pipe is in bytes: need to convert to string)

            msg_list = resp[1].decode("utf-8").split("\n")
            topic_name = msg_list[0]
            if msg_list[1] == "rosgraph_msgs/Clock":
                if msg_list[0] not in object_class.pub_dict:
                    object_class.create_publisher(topic_name, Clock)
                sim_time_msg = json_message_converter.convert_json_to_ros_message('rosgraph_msgs/Clock', msg_list[2])
                object_class.pub_dict[topic_name].publish(sim_time_msg)

            elif msg_list[1] == "nav_msgs/Odometry":
                if msg_list[0] not in object_class.pub_dict:
                    object_class.create_publisher(topic_name, Odometry)
                odom_msg = json_message_converter.convert_json_to_ros_message('nav_msgs/Odometry', msg_list[2])
                object_class.pub_dict[topic_name].publish(odom_msg)

            elif msg_list[1] == "sensor_msgs/LaserScan":
                if msg_list[0] not in object_class.pub_dict:
                    object_class.create_publisher(topic_name, LaserScan)
                scan_msg = json_message_converter.convert_json_to_ros_message('sensor_msgs/LaserScan', msg_list[2])
                for count, value in enumerate(scan_msg.ranges):
                    if float(value) == 1000:
                        scan_msg.ranges[count] = float('Inf')
                object_class.pub_dict[topic_name].publish(scan_msg)
                
            elif msg_list[1] == "sensor_msgs/Imu":
                if msg_list[0] not in object_class.pub_dict:
                    object_class.create_publisher(topic_name, Imu)
                imu_msg = json_message_converter.convert_json_to_ros_message('sensor_msgs/Imu', msg_list[2])
                object_class.pub_dict[topic_name].publish(imu_msg)

            elif msg_list[1] == "tf2_msgs/TFMessage":
                if msg_list[0] not in object_class.pub_dict:
                    object_class.create_publisher(topic_name, TFMessage)
                tf_msg = json_message_converter.convert_json_to_ros_message('tf2_msgs/TFMessage', msg_list[2])
                object_class.pub_dict[topic_name].publish(tf_msg)


    except rospy.ROSInterruptException:
        sys.exit(1)

    except pywintypes.error as e:
        errno, _, errstr = e.args
        if errno == 109:
            print(f"Lost connection: {errstr}")
        else:
            raise

    except:
        raise
