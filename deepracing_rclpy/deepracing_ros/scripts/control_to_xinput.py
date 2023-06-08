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

from typing import List
import rclpy
import rclpy.parameter
import rclpy.timer
import rclpy.qos
import rclpy.exceptions
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String, Float64
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ackermann_msgs.msg import AckermannDriveStamped
from control_msgs.msg import PidState
from deepracing_msgs.msg import XinputState
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import json
from tqdm import tqdm
import numpy as np
import scipy.interpolate

class ControlToXinputNode(Node):
    def __init__(self, name="control_to_xinput"):
        super(ControlToXinputNode, self).__init__(name)
        self.current_ackermann_data : AckermannDriveStamped = None
        self.ackermann_sub : Subscription = self.create_subscription(AckermannDriveStamped, "ctrl_cmd", self.ackermannCB, 1)
        self.pid_state_sub : Subscription = \
            self.create_subscription(PidState, "velocity_pid_state", self.pidstateCB, rclpy.qos.qos_profile_sensor_data)
        
        self.xinput_publisher : Publisher = self.create_publisher(XinputState, "controller_input", 1)
        
        steering_angles_desc : ParameterDescriptor = ParameterDescriptor()
        steering_angles_desc.name = "steering_angles"
        steering_angles_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        steering_angles_param : rclpy.Parameter = self.declare_parameter(steering_angles_desc.name, descriptor=steering_angles_desc)
        
        steering_angles : List[float] = steering_angles_param.get_parameter_value().double_array_value
        if len(steering_angles)==0:
            raise rclpy.exceptions.InvalidParameterValueException("Must set \"steering_angles\" parameter to a non-empty list of floats")
        steering_angles_fit : np.ndarray = np.asarray(steering_angles)
        
        
        control_values_desc : ParameterDescriptor = ParameterDescriptor()
        control_values_desc.name = "control_values"
        control_values_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        control_values_param : rclpy.Parameter = self.declare_parameter(control_values_desc.name, descriptor=control_values_desc)
        
        control_values : List[float] = control_values_param.get_parameter_value().double_array_value
        if len(control_values)==0:
            raise rclpy.exceptions.InvalidParameterValueException("Must set \"control_values\" parameter to a non-empty list of floats")
        control_values_fit : np.ndarray = np.asarray(control_values)

        if not (steering_angles_fit.shape[0]==control_values_fit.shape[0]):
            raise rclpy.exceptions.InvalidParameterValueException("Got %u values in \"steering_angles\", but %u values in \"control_values\"" % (steering_angles_fit.shape[0],control_values_fit.shape[0]))

        Inonzero : np.ndarray = np.abs(steering_angles_fit)>0.0
        steering_angles_fit = steering_angles_fit[Inonzero]
        control_values_fit = control_values_fit[Inonzero]

        steering_angles_fit, Iunique  = np.unique(steering_angles_fit, return_index=True)
        control_values_fit = control_values_fit[Iunique]

        Isort : np.ndarray = np.argsort(steering_angles_fit)
        self.steering_angles_fit = steering_angles_fit[Isort]
        self.control_values_fit = control_values_fit[Isort]

        self.largest_negative : float = float(np.max(self.steering_angles_fit[self.steering_angles_fit<0.0]))
        self.smallest_positive : float = float(np.min(self.steering_angles_fit[self.steering_angles_fit>0.0]))

        spline_order_param : rclpy.Parameter = self.declare_parameter("spline_order", value=1)
        spline_order = spline_order_param.get_parameter_value().integer_value
        self.spline : scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(self.steering_angles_fit, self.control_values_fit, k=spline_order)


        
        
    def ackermannCB(self, data : AckermannDriveStamped):
        self.current_ackermann_data = data

    def pidstateCB(self, data : PidState):
        if self.current_ackermann_data is None:
            # self.get_logger().error("Can't publish control, haven't received any steering data yet")
            return
        state : XinputState = XinputState()

        if data.output>=0.0:
            state.gamepad.right_trigger = int(np.round(np.clip(data.output*255.0, 0.0, 255.0)))
        else:
            state.gamepad.left_trigger = int(np.round(np.clip(data.output*255.0, 0.0, 255.0)))
        
        steering : float = self.current_ackermann_data.drive.steering_angle

        if steering<self.largest_negative or steering>self.smallest_positive:
            state.gamepad.thumb_lx = int(np.clip(np.round(self.spline(steering)), -32768.0, 32767.0))
        else:
            state.gamepad.thumb_lx = 0
        
        self.xinput_publisher.publish(state)
        

        
        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    spinner : MultiThreadedExecutor = MultiThreadedExecutor(num_threads=2)
    node = ControlToXinputNode()
    spinner.add_node(node)
    spinner.spin()
    rclpy.shutdown()


    


if __name__ == '__main__':
    main()