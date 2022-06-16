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
import rclpy.client
import rclpy.executors
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String, Float64
from nav_msgs.msg import Path
import deepracing_msgs.srv
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData, CarControl, BezierCurve
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.path_server_oracle_raceline import OraclePathServer
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor
class RacelineSetter(Node):
    def __init__(self, name="initial_raceline_setter"):
        super(RacelineSetter, self).__init__(name)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = RacelineSetter()
    default_trackname_param : rclpy.Parameter = node.declare_parameter("default_trackfile", value="")
    default_trackname : str = default_trackname_param.get_parameter_value().string_value
    if default_trackname=="":
        exit(0)
    serviceclient : rclpy.client.Client = node.create_client(deepracing_msgs.srv.SetRaceline, "set_raceline")
    serviceclient.wait_for_service()
    req : deepracing_msgs.srv.SetRaceline.Request = deepracing_msgs.srv.SetRaceline.Request()
    req.filename=default_trackname
    serviceclient.call_async(req)
    rclpy.spin_once(node)
if __name__ == '__main__':
    main()