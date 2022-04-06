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

from copy import deepcopy
import rclpy
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from deepracing_msgs.msg import BezierCurve
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, TransformStamped, Transform
from autoware_auto_msgs.msg import VehicleControlCommand
import numpy as np
import rclpy.executors
from deepracing_ros.utils import AsyncSpinner
from scipy.spatial.transform import Rotation
import deepracing_ros, deepracing_ros.convert as C
from typing import List
import torch
import deepracing_models.math_utils as mu
import tf2_ros
import rclpy.time
import rclpy.duration


class BezierCurvePurePursuit(Node):
    def __init__(self,):
        super(BezierCurvePurePursuit,self).__init__('bezier_pure_pursuit')
        self.setpoint_pub : Publisher = self.create_publisher(VehicleControlCommand, "ctrl_cmd", 1)
        self.curve_sub : Subscription = self.create_subscription(BezierCurve, "beziercurves_in", self.curveCB, 1)
        self.odom_sub : Subscription = self.create_subscription(Odometry, "odom", self.odomCB, 1)
        self.current_curve_msg : BezierCurve = None

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)

        lookahead_gain_param : Parameter = self.declare_parameter("lookahead_gain", value=0.4)
        self.lookahead_gain : float = lookahead_gain_param.get_parameter_value().double_value

        velocity_lookahead_gain_param : Parameter = self.declare_parameter("velocity_lookahead_gain", value=0.175)
        self.velocity_lookahead_gain : float = velocity_lookahead_gain_param.get_parameter_value().double_value

        wheelbase_param : Parameter = self.declare_parameter("wheelbase", value=3.05)
        wheelbase : float = wheelbase_param.get_parameter_value().double_value

        gpu_param : Parameter = self.declare_parameter("gpu", value=-1)
        gpu : int = gpu_param.get_parameter_value().integer_value

        if torch.has_cuda and gpu>=0:
            self.get_logger().info("Running on GPU %d" %(gpu,))
            self.device : torch.device = torch.device("cuda:%d" % (gpu,))
        else:
            self.get_logger().info("Running on the CPU")
            self.device : torch.device = torch.device("cpu")


        self.tsamp : torch.Tensor = torch.linspace(0.0, 1.0, 400, dtype=torch.float32, device=self.device).unsqueeze(0)
        self.twoL : torch.Tensor = torch.as_tensor(2.0*wheelbase, dtype=self.tsamp.dtype, device=self.tsamp.device)



    def curveCB(self, curve_msg : BezierCurve):
        self.current_curve_msg = curve_msg
    def odomCB(self, odom_msg : Odometry):
        if self.current_curve_msg is None:
            return
        current_curve : BezierCurve = deepcopy(self.current_curve_msg)
        
        map_to_car : torch.Tensor = C.poseMsgToTorch(odom_msg.pose.pose, dtype=self.tsamp.dtype, device=self.tsamp.device)

        if not odom_msg.child_frame_id=="base_link":
            car_to_base_link_msg : TransformStamped = self.tf2_buffer.lookup_transform(odom_msg.child_frame_id, "base_link", rclpy.time.Time.from_msg(odom_msg.header.stamp), rclpy.duration.Duration(seconds=2))
            car_to_base_link : torch.Tensor = C.transformMsgToTorch(car_to_base_link_msg.transform, dtype=map_to_car.dtype, device=map_to_car.device)
            pose_curr : torch.Tensor = torch.matmul(map_to_car, car_to_base_link)
        else:
            pose_curr : torch.Tensor = map_to_car

        transform : torch.Tensor = torch.inverse(pose_curr)

        bcurve_global : torch.Tensor = torch.ones(len(current_curve.control_points), 4, dtype=map_to_car.dtype, device=map_to_car.device )
        for i in range(bcurve_global.shape[0]):
            bcurve_global[i,0]=current_curve.control_points[i].x
            bcurve_global[i,1]=current_curve.control_points[i].y
            bcurve_global[i,2]=current_curve.control_points[i].z
        bcurve_local = torch.matmul(bcurve_global, transform[0:3].t()).unsqueeze(0)
        dt : float = (float(current_curve.delta_t.sec) + float(current_curve.delta_t.nanosec)*1E-9)
        
        Msamp : torch.Tensor = mu.bezierM(self.tsamp, bcurve_local.shape[1]-1)
        Psamp : torch.Tensor = torch.matmul(Msamp, bcurve_local)
        Psamp=Psamp[0]
        arclengths : torch.Tensor = torch.zeros_like(self.tsamp[0])
        arclengths[1:]=torch.cumsum(torch.norm(Psamp[1:] - Psamp[:-1], p=2, dim=1), 0)

        _, _velocities = mu.bezierDerivative(bcurve_local, t=self.tsamp)
        velocities : torch.Tensor = _velocities[0]/dt
        speeds : torch.Tensor = torch.norm(velocities, p=2, dim=1)
        unit_tangents : torch.Tensor = velocities/speeds[:,None]

        _, _accelerations = mu.bezierDerivative(bcurve_local, t=self.tsamp, order=2)
        accelerations : torch.Tensor = _accelerations[0]/(dt*dt)
        longitudinal_accelerations : torch.Tensor = torch.sum(accelerations*unit_tangents, dim=1)



        idx = torch.argmin(torch.norm(Psamp, p=2, dim=1))
        tsamp = self.tsamp[:,idx:]
        velocities = velocities[idx:]
        longitudinal_accelerations = longitudinal_accelerations[idx:]
        speeds = speeds[idx:]
        Psamp = Psamp[idx:]
        arclengths = arclengths[idx:]
        arclengths = arclengths - arclengths[0]

        vel_local = odom_msg.twist.twist.linear
        current_speed : float = torch.norm(torch.as_tensor([vel_local.x, vel_local.y, vel_local.z]), p=2).item()
        lookahead_distance = max(self.lookahead_gain*current_speed, 10.0)
        if current_speed>55.0:
            lookahead_distance_vel = self.velocity_lookahead_gain*current_speed
        else:
            lookahead_distance_vel=0.00
        lookahead_distance_vel = self.velocity_lookahead_gain*current_speed

        lookahead_index = torch.argmin(torch.abs(arclengths-lookahead_distance))
        lookahead_index_vel = torch.argmin(torch.abs(arclengths-lookahead_distance_vel))

        lookaheadVector = Psamp[lookahead_index]
        ld = torch.norm(lookaheadVector, p=2)
        lookaheadDirection = lookaheadVector/ld
        alpha = torch.atan2(lookaheadDirection[1],lookaheadDirection[0])

        control_out : VehicleControlCommand = VehicleControlCommand(stamp=odom_msg.header.stamp)
        control_out.front_wheel_angle_rad = torch.atan((self.twoL * torch.sin(alpha)) / ld).item()
        control_out.velocity_mps=speeds[lookahead_index_vel].item()
        control_out.long_accel_mps2=longitudinal_accelerations[lookahead_index_vel].item()

        self.setpoint_pub.publish(control_out)

        


    
def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BezierCurvePurePursuit()
    num_threads_param : rclpy.Parameter = node.declare_parameter("num_threads", 0)
    num_threads : int = num_threads_param.get_parameter_value().integer_value
    if num_threads<=0:
        node.get_logger().info("Spinning with number of CPU cores")
        spinner : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor(None)
    else:
        node.get_logger().info("Spinning with %d threads" % (num_threads,))
        spinner : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor(num_threads)
    spinner.add_node(node)
    try:
        spinner.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()