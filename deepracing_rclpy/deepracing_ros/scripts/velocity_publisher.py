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

import rclpy, rclpy.clock, rclpy.time
from rclpy.node import Node
from rclpy import Parameter
from std_msgs.msg import String
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point, Point32, Vector3Stamped
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import PointCloud2, PointCloud, PointField, PointField
from std_msgs.msg import Header
import deepracing, deepracing.protobuf_utils as proto_utils

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.sub = self.create_subscription(TimestampedPacketMotionData, '/motion_data', self.motion_data_callback, 1)
        self.vel_pub = self.create_publisher(Vector3Stamped, '/car_velocity', 1)
        
    def motion_data_callback(self, msg):
        motion_data : CarMotionData  = msg.udp_packet.car_motion_data[msg.udp_packet.header.player_car_index]
        self.vel_pub.publish(motion_data.world_velocity)

        


def main(args=None):
    rclpy.init(args=args)

    node = VelocityPublisher()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=1)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()