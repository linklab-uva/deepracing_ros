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
from rclpy.publisher import Publisher
from std_msgs.msg import String, Float64
from nav_msgs.msg import Path
from autoware_auto_msgs.msg import Trajectory
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData, CarControl
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.pure_puresuit_control_ros import PurePursuitControllerROS
from deepracing_ros.controls.pure_puresuit_control_oracle_raceline import OraclePurePursuitControllerROS
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    spinner : AsyncSpinner = AsyncSpinner(MultiThreadedExecutor())
    node = OraclePurePursuitControllerROS()
    path_pub : Publisher = node.create_publisher(Path, "localpaths", 1)
    traj_pub : Publisher = node.create_publisher(Trajectory, "localtrajectories", 1)
    spinner.add_node(node)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    spinner.spin()
    rate : rclpy.timer.Rate = node.create_rate(30.0)
    try:
        while rclpy.ok():
            rate.sleep()
            _, path_msg, traj_msg = node.getTrajectory()
            if (path_msg is not None) and (traj_msg is not None):
                path_pub.publish(path_msg)
                traj_pub.publish(traj_msg)
    except KeyboardInterrupt:
        pass
    spinner.shutdown()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()