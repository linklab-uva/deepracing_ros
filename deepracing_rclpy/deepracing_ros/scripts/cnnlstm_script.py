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
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.cnnlstm_ros import CNNLSTMROS
from rclpy import executors
from deepracing_ros.utils import AsyncSpinner
from deepracing_msgs.msg import CarControl

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = CNNLSTMROS()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    control_pub = node.create_publisher( CarControl, '/car_control', 1)
    timer = node.create_rate(30.0)
    spinner = AsyncSpinner(executor=executors.MultiThreadedExecutor(3))
    spinner.addNode(node)
    spinner.spin()
    try:
        while True:
            control = node.getControl()
            if control is not None:
                control_pub.publish(control)
    except KeyboardInterrupt:
        pass
    node.stop()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()