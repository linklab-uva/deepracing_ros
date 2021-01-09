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
import rclpy.executors as executors
from deepracing_ros.utils import AsyncSpinner
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.pure_puresuit_control_ros import PurePursuitControllerROS
from deepracing_ros.controls.pure_puresuit_control_bezier_predictor import AdmiralNetBezierPurePursuitControllerROS
from deepracing_ros.controls.pure_puresuit_control_probabilistic_bezier import ProbabilisticBezierPurePursuitControllerROS

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    spinner = AsyncSpinner(executor=executors.MultiThreadedExecutor(3))
   # node = AdmiralNetBezierPurePursuitControllerROS()
    node = ProbabilisticBezierPurePursuitControllerROS()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    control_pub = node.create_publisher(CarControl, "/car_control", 1)
    spinner.add_node(node)
    spinner.spin()
    rate = node.create_rate(60.0)

    try:
        while rclpy.ok():
            cc = node.getControl()["control"]
            if cc is not None:
                control_pub.publish(cc)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()