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
import rclpy.executors
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  
import rclpy.logging  
import robot_localization.msg
import deepracing_msgs.msg  
import numpy as np
from scipy.spatial.transform import Rotation
import deepracing_ros.utils
import deepracing_ros.convert
import rclpy.executors 
import copy
class EKFMonitor(rclpy.node.Node):
    def __init__(self, name="ekf_monitor"):
        super(EKFMonitor, self).__init__(name)
        self.motion_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketMotionData, "motion_data", self.motionDataCB, 10)
        self.set_state_pub : rclpy.publisher.Publisher = self.create_publisher(robot_localization.msg.State, "set_state", 1)
        self.prev_motion_data : deepracing_msgs.msg.TimestampedPacketMotionData = None
    def motionDataCB(self, motion_data : deepracing_msgs.msg.TimestampedPacketMotionData):
        if self.prev_motion_data is None:
            self.prev_motion_data = copy.deepcopy(motion_data)
            return        
        self.prev_motion_data = copy.deepcopy(motion_data)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = EKFMonitor()
    rclpy.spin(node, rclpy.executors.MultiThreadedExecutor())

if __name__ == '__main__':
    main()