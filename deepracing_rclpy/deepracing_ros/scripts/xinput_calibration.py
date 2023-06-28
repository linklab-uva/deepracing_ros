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
from rclpy.subscription import Subscription
from std_msgs.msg import String, Float64
from deepracing_msgs.msg import XinputState, XinputGamepad, TimestampedPacketMotionData
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import json
from tqdm import tqdm

class CalibrationNode(Node):
    def __init__(self,):
        super(CalibrationNode,self).__init__('calibration_node')
        self.current_motion_data : TimestampedPacketMotionData = None
        self.subscriber : Subscription = self.create_subscription(TimestampedPacketMotionData, "motion_data", self.dataCB_, 1)
    def dataCB_(self, data : TimestampedPacketMotionData):
        self.current_motion_data = data
def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    spinner : AsyncSpinner = AsyncSpinner(MultiThreadedExecutor(num_threads=3))
    node = CalibrationNode()
    xinput_pub : Publisher = node.create_publisher(XinputState, "controller_override", 1)
    spinner.add_node(node)
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    spinner.spin()
    sleeptime_param : rclpy.Parameter = node.declare_parameter("sleeptime", value=0.5)
    control_axis_param : rclpy.Parameter = node.declare_parameter("control_axis", value="")
    control_axis : bool = control_axis_param.get_parameter_value().string_value
    one_second_rate : rclpy.timer.Rate = node.create_rate(1.0)
    rate : rclpy.timer.Rate = node.create_rate(1.0/(sleeptime_param.get_parameter_value().double_value))
    state : XinputState = XinputState()
    xinput_pub.publish(state)
    min = -32768.0
    max = 32767.0
    if control_axis in {"left", "right"}:
        node.get_logger().info("Calibrating in 3")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 2")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 1")
        one_second_rate.sleep()
        xinput_pub.publish(state)
        one_second_rate.sleep()
    if control_axis=="left":
        steps = np.linspace(0.0, min, num=64)
        for i in range(steps.shape[0]):
            state.gamepad.thumb_lx = int(round(steps[i]))
            xinput_pub.publish(state)
            rate.sleep()
        one_second_rate.sleep()
        steps = np.linspace(min, 0.0, num=64)
        for i in range(steps.shape[0]):
            state.gamepad.thumb_lx = int(round(steps[i]))
            xinput_pub.publish(state)
            rate.sleep()
        node.get_logger().info("Done")
    elif control_axis=="right":
        steps = np.linspace(0.0, max, num=64)
        for i in range(steps.shape[0]):
            state.gamepad.thumb_lx = int(round(steps[i]))
            xinput_pub.publish(state)
            rate.sleep()
        one_second_rate.sleep()
        steps = np.linspace(max, 0.0, num=64)
        for i in range(steps.shape[0]):
            state.gamepad.thumb_lx = int(round(steps[i]))
            xinput_pub.publish(state)
            rate.sleep()
        node.get_logger().info("Done")
    else:
        while node.current_motion_data is None:
            rate.sleep()
        steps = np.concatenate([
            np.linspace(min, -7000.0, num=20), 
            np.linspace(-6000.0, 6000.0, num=20), 
            np.linspace(7000.0, max, num=20)], 
            axis=0).round()
        state.gamepad.thumb_lx = int(round(steps[0]))
        xinput_pub.publish(state)
        node.get_logger().info("Calibrating in 5")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 4")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 3")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 2")
        one_second_rate.sleep()
        node.get_logger().info("Calibrating in 1")
        one_second_rate.sleep()
        wheel_vals = np.zeros_like(steps)
        # rate.sleep()
        # rate.sleep()
        for i in tqdm(range(steps.shape[0])):
            state.gamepad.thumb_lx = int(round(steps[i]))
            xinput_pub.publish(state)
            rate.sleep()
            wheel_vals[i] = -node.current_motion_data.udp_packet.front_wheels_angle
        print(steps)
        print(wheel_vals)
        
        figure = plt.figure()
        plt.plot(wheel_vals, steps)
        figure2 = plt.figure()
        plt.plot(steps, wheel_vals)
        plt.show()

        d : dict = dict()
        d["xinput_values"] = steps.tolist()
        d["steering_wheel_angles"] = wheel_vals.tolist()
        with open("vigem_calibration.json", "w") as f:
            json.dump(d, f, indent=2)

    spinner.shutdown()
    rclpy.shutdown()


    


if __name__ == '__main__':
    main()