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
    spinner : AsyncSpinner = AsyncSpinner(MultiThreadedExecutor())
    node = OraclePathServer()
    bcurve_pub : Publisher = node.create_publisher(BezierCurve, "oraclebeziercurves", 1)
    spinner.add_node(node)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    spinner.spin()
    frequency_param : rclpy.Parameter = node.declare_parameter("rate", value=10.0)
    rate : rclpy.timer.Rate = node.create_rate(frequency_param.get_parameter_value().double_value)
    try:
        while rclpy.ok():
            rate.sleep()
            bcurve_msg = node.getTrajectory()
            if bcurve_msg is not None:
                bcurve_pub.publish(bcurve_msg)
    except KeyboardInterrupt:
        pass
    spinner.shutdown()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()