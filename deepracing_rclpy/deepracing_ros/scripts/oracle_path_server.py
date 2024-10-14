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
import rclpy.timer
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String, Float64
from nav_msgs.msg import Path
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData, CarControl, BezierCurve
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.path_server_oracle_raceline import OraclePathServer
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = OraclePathServer()
    # spinner : AsyncSpinner = AsyncSpinner(MultiThreadedExecutor())
    # bcurve_pub : Publisher = node.create_publisher(BezierCurve, "oraclebeziercurves", 1)
    # spinner.add_node(node)
    # spinner.spin()
    frequency_param : rclpy.Parameter = node.declare_parameter("rate", value=100.0)
    timer : rclpy.timer.Timer = node.create_timer(1.0/frequency_param.get_parameter_value().double_value, node.getTrajectory)

    rclpy.spin(node)
    


if __name__ == '__main__':
    main()