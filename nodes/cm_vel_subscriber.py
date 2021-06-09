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

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
import sys
import fileinput
import time
import win32pipe, win32file, pywintypes
import os


import time

try:
    from memorpy import *
except:
    print ("Make sure you have memorpy installed")
    sys.exit(1)


class SharedMemorySegment(object):
    def __init__(self, process_name='FarmingSimulator2019Game.exe',
        marker_start='fs_lua_memorp_buffer_0012', marker_end='fs_lua_memorp_buffer_end_0012',
        buflen_len=1, expected_size=None
    ):
        self.__buflen_len = buflen_len
        self.__mw = MemWorker(name=process_name)
        # sentinel values
        self.__marker_start = marker_start
        self.__marker_end = marker_end

        # search for sentinel start value
        print ("Searching start sentinel ('{}')".format(self.__marker_start))
        offsets = [x for x in self.__mw.mem_search(str.encode(self.__marker_start))]
        if not offsets:
            raise ValueError("No sentinels found.")
        # print("Found {} candidate offsets: {}".format(len(offsets), ', '.join([hex(o) for o in offsets])))
        hex_offsets = [o.__hex__ for o in offsets]
       
        print("Found {} candidate offsets: {}".format(len(offsets), hex_offsets))
       
        # only care about a single buffer (for now)
        # TODO: support some sort of buffer ID or name so we can attach
        # to a specific instance instead of just the first one we find
        self.__base_offset = self.__find_input_buffer(offsets)
        self.__len_offset = self.__base_offset + len(self.__marker_start)
        self.__data_offset = self.__len_offset + buflen_len

        # determine size of buffer as configured on the Lua side
        self.__len = ord(self.__len_offset.read(maxlen=buflen_len))

        if expected_size and expected_size != self.__len:
            raise ValueError("Found buffer not of expected size (found {}, "
                "expected {} bytes)".format(self.__len, expected))

        # # debug outputs
        # print ("Base offset: {}".format(hex(self.__base_offset)))
        # print ("Length offset: {}".format(hex(self.__len_offset)))
        # print ("Data offset {}".format(hex(self.__data_offset)))
        # print ("Lua configured length: {} bytes".format(self.__len))
   

    def __find_input_buffer(self, offsets):
        # iterate, stop at first match
        for offset in offsets:
            if self.__is_likely_buffer(offset):
                return offset
        raise ValueError("No buffer found")


    def __is_likely_buffer(self, candidate_offset):
        # TODO: maybe make this actually return either None, or a tuple with
        # the offsets and length of the buffer at 'candidate_offset'
        len_offset = candidate_offset + len(self.__marker_start)
        data_offset = len_offset + self.__buflen_len

        # length is encoded as single byte in the string
        len_ = ord(len_offset.read(self.__buflen_len))

        # arbitrary limit, but something we could expect
        if len_ < 0:
            raise ValueError("Unexpected length of buffer (got {})".format(len_))

        # we should find the end marker at the end of the buffer
        end_offset = data_offset + len_
        mb_marker_end = end_offset.read(maxlen=len(self.__marker_end), _type='bytes')
        #print ("is_likely_buffer: at end: '{}'".format(mb_marker_end))
        return mb_marker_end == str.encode(self.__marker_end)


    def read(self, len_=None):
        return self.__data_offset.read(maxlen=len_ or self.__len, _type='bytes')


    def write(self, data, offset=0):
        (self.__data_offset + offset).write(data, _type='bytes')


    def clear(self, fill_char=' '):
        self.write(data=str.encode(fill_char*self.__len))


DEFAULT_PROCESS_NAME='FarmingSimulator2019Game.exe'

import argparse


parser = argparse.ArgumentParser()
parser.add_argument('-p', '--process-name', type=str, dest='process_name',
    metavar='PROCESS', help="Process to search for in-memory buffers in.",
    default=DEFAULT_PROCESS_NAME)


args = parser.parse_args()

print ("Searching for SharedMemorySegments in '{}'".format(args.process_name))
buf = SharedMemorySegment(process_name=args.process_name)

print ("Current buffer contents: '{}'".format(buf.read().decode('ascii')))
print ("Pushing data to Lua ..")



#  subscribes the joystick state for the input to control vehicles in Farming Simulator.  
def cmd_vel_callback(data):
    
    acc_rate = 1
    acc = acc_rate*data.linear.x
    rotatedTime_param = data.angular.z
    
    if acc == 0:
        allowedToDrive = "false"
    else:
        allowedToDrive = "true"

    
    byte_str = str.encode("{} {} {}".format(acc, rotatedTime_param, allowedToDrive))
    buf.write(data= byte_str)
 

def cml_vel_subscriber():


    rospy.init_node('cml_vel_subscriber', anonymous=True)
    
   
    rospy.loginfo('Waiting for topic %s to be published...','cmd_vel')
    rospy.wait_for_message('cmd_vel', Twist)
    rospy.loginfo('%s topic is now available!', 'cmd_vel')


    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    cml_vel_subscriber()
    buf.clear()
