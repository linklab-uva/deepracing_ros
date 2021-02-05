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
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from deepracing_msgs.msg import CarControl, CarStatusData, CarTelemetryData, TimestampedPacketCarStatusData, TimestampedPacketCarTelemetryData
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.pilotnet_ros import PilotNetROS
import rpyutils
with rpyutils.add_dll_directories_from_env("PATH"):
    import py_f1_interface
class VJoyControl(Node):
    def __init__(self,):
        super(VJoyControl,self).__init__('vjoy_control')
        qos = QoSProfile(depth=1)
        self.vjoy_index = self.declare_parameter("vjoy_index", value=1)
        self.control_sub = self.create_subscription(CarControl, "/car_control", self.controlCB, qos)
        self.controller = py_f1_interface.F1Interface(self.vjoy_index.get_parameter_value().integer_value)
        self.controller.setControl(0.0,0.0,0.0)      

        self.status_sub = self.create_subscription(TimestampedPacketCarStatusData, "/cropped_publisher/status_data", self.statusPacketCB, 1)
        self.telemery_sub = self.create_subscription(TimestampedPacketCarTelemetryData, "/cropped_publisher/telemetry_data", self.telemeryPacketCB, 1)

        self.current_status_packet : CarStatusData = CarStatusData()
        self.current_telemetry_packet : CarTelemetryData = CarTelemetryData()
        
    
    def statusPacketCB(self, status_packet: TimestampedPacketCarStatusData):
        ego_idx = status_packet.udp_packet.header.player_car_index
        self.current_status_packet = status_packet.udp_packet.car_status_data[ego_idx]
        
    def telemeryPacketCB(self, telemetry_packet: TimestampedPacketCarTelemetryData):
        ego_idx = telemetry_packet.udp_packet.header.player_car_index
        self.current_telemetry_packet = telemetry_packet.udp_packet.car_telemetry_data[ego_idx]

    def controlCB(self, control : CarControl):
        self.controller.setControl(control.steering, control.throttle, control.brake)
        # if self.current_status_packet.drs_allowed and (not bool(self.current_telemetry_packet.drs)):
        #     self.controller.pushDRS()

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = VJoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()