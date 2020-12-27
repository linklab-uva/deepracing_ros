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

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from rclpy.node import Rate
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ROSClock
from deepracing_ros.utils import AsyncSpinner
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_py
import numpy as np
import scipy
from scipy.spatial.transform.rotation import Rotation as Rot
import deepracing_ros.convert as C
import torch

def main(args=None):
    rclpy.init(args=args)

    node = Node("tf2_test")
    frame1param = node.declare_parameter("frame1")
    frame2param = node.declare_parameter("frame2")
    frame1 = frame1param.get_parameter_value().string_value
    frame2 = frame2param.get_parameter_value().string_value
    tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(node=node)
    tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(tf2_buffer,node)
    mt_ex : AsyncSpinner = AsyncSpinner(rclpy.executors.MultiThreadedExecutor(3))
    mt_ex.addNode(node)
    mt_ex.spin()
    clock = Clock()
    rate = node.create_rate(1.0)
    while rclpy.ok():
        t : Time = clock.now()
        transform : TransformStamped = tf2_buffer.lookup_transform(frame1, frame2, t, timeout=Duration(seconds=4))
        transformtorch = C.transformMsgToTorch(transform.transform, dtype=torch.float64)
        transforminv : TransformStamped = tf2_buffer.lookup_transform(frame2, frame1, t, timeout=Duration(seconds=4))
        transforminvtorch = C.transformMsgToTorch(transforminv.transform, dtype=torch.float64)
        print()
        print(t)
        print(transformtorch)
        print(transforminvtorch)
        mm = torch.matmul(transformtorch, transforminvtorch)
        mask = (torch.abs(mm)>1E-10).type(mm.dtype)
        print(mm*mask)
        print(transforminvtorch - torch.inverse(transformtorch))
        print()
        rate.sleep()


if __name__ == '__main__':
    main()