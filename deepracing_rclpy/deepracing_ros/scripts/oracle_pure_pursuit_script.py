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
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData, CarControl
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.pure_puresuit_control_ros import PurePursuitControllerROS
from deepracing_ros.controls.pure_puresuit_control_oracle_trackfile import OraclePurePursuitControllerROS
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    spinner : AsyncSpinner = AsyncSpinner(MultiThreadedExecutor(3))
    node = OraclePurePursuitControllerROS()
    control_pub = node.create_publisher(CarControl, "/car_control", 1)
    spinner.addNode(node)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    spinner.spin()
    rate : rclpy.timer.Rate = node.create_rate(60.0)
    try:
        while rclpy.ok():
            rate.sleep()
            control : CarControl = node.getControl()
            if control is not None:
                control_pub.publish(control)
    except KeyboardInterrupt:
        pass
    spinner.shutdown()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()