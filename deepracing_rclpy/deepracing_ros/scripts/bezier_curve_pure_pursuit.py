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
from multiprocessing import Semaphore
import rclpy
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from deepracing_msgs.msg import BezierCurve, TimestampedPacketSessionData
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, Point, TransformStamped, Transform
from ackermann_msgs.msg import AckermannDriveStamped
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
import threading 
import deepracing_ros.convert as C
import std_msgs.msg


class BezierCurvePurePursuit(Node):
    def __init__(self,):
        super(BezierCurvePurePursuit,self).__init__('bezier_pure_pursuit')
        self.setpoint_pub : Publisher = self.create_publisher(AckermannDriveStamped, "ctrl_cmd", 1)
        self.lateral_error_pub : Publisher = self.create_publisher(Float64, "lateral_error", 1)
        self.local_curve_pub : Publisher = self.create_publisher(BezierCurve, "local_bezier_curves", 1)
        self.curve_sub : Subscription = self.create_subscription(BezierCurve, "beziercurves_in", self.curveCB, 1)

        self.odom_sub : Subscription = self.create_subscription(Odometry, "odom", self.odomCB, 1)
        self.session_sub : Subscription = self.create_subscription(TimestampedPacketSessionData, "/session_data", self.sessionCB, 1)
        self.current_curve_msg : BezierCurve = None
        self.current_curve_mutex : threading.Semaphore = threading.Semaphore()

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)

        carname_param : Parameter = self.declare_parameter("carname", value="")
        self.carname : str = carname_param.get_parameter_value().string_value

        self.base_link_id : str = "base_link_%s" % (self.carname,)

        lookahead_gain_param : Parameter = self.declare_parameter("lookahead_gain", value=0.4)
        self.lookahead_gain : float = lookahead_gain_param.get_parameter_value().double_value

        velocity_lookahead_gain_param : Parameter = self.declare_parameter("velocity_lookahead_gain", value=0.2)
        self.velocity_lookahead_gain : float = velocity_lookahead_gain_param.get_parameter_value().double_value

        wheelbase_param : Parameter = self.declare_parameter("wheelbase", value=3.726)
        wheelbase : float = wheelbase_param.get_parameter_value().double_value

        gpu_param : Parameter = self.declare_parameter("gpu", value=-1)
        gpu : int = gpu_param.get_parameter_value().integer_value

        if torch.has_cuda and gpu>=0:
            self.get_logger().info("Running on GPU %d" %(gpu,))
            self.device : torch.device = torch.device("cuda:%d" % (gpu,))
        else:
            self.get_logger().info("Running on the CPU")
            self.device : torch.device = torch.device("cpu")


        self.tsamp : torch.Tensor = torch.linspace(0.0, 1.0, 400, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.twoL : torch.Tensor = torch.as_tensor(2.0*wheelbase, dtype=self.tsamp.dtype, device=self.tsamp.device)

        self.player_car_index : int = 0



    def sessionCB(self, session_msg : TimestampedPacketSessionData):
        self.player_car_index = session_msg.udp_packet.header.player_car_index

    def curveCB(self, curve_msg : BezierCurve):
        if not self.current_curve_mutex.acquire(timeout=0.5):
            self.get_logger().error("Unable to acquire current_curve_mutex")
            return
        self.current_curve_msg = curve_msg
        self.current_curve_mutex.release()

    def odomCB(self, odom_msg : Odometry):
        if self.current_curve_msg is None:
            self.get_logger().error("No bezier curve received yet")
            return
        if not self.current_curve_mutex.acquire(timeout=0.5):
            self.get_logger().error("Unable to acquire current_curve_mutex")
            return
        current_curve : BezierCurve = deepcopy(self.current_curve_msg)
        self.current_curve_mutex.release()
        
        
        map_to_car : torch.Tensor = C.poseMsgToTorch(odom_msg.pose.pose, dtype=self.tsamp.dtype, device=self.tsamp.device)
        try:
            if not odom_msg.child_frame_id==self.base_link_id:
                car_to_base_link_msg : TransformStamped = self.tf2_buffer.lookup_transform(odom_msg.child_frame_id, self.base_link_id, rclpy.time.Time.from_msg(odom_msg.header.stamp), rclpy.duration.Duration(seconds=2))
                car_to_base_link : torch.Tensor = C.transformMsgToTorch(car_to_base_link_msg.transform, dtype=map_to_car.dtype, device=map_to_car.device)
                pose_curr : torch.Tensor = torch.matmul(map_to_car, car_to_base_link)
            else:
                pose_curr : torch.Tensor = map_to_car
        except Exception as e:
            self.get_logger().error("Unable to lookup TF2 transform for base link frame: %s." % (self.base_link_id,))
            return
        
        # transform : torch.Tensor = torch.eye(4, device=pose_curr.device, dtype=pose_curr.dtype)
        # transform[0:3] = pose_curr[0:3].t()
        # transform[0:3,3] = torch.matmul(transform[0:3], -pose_curr[0:3,3])
        Rinv = pose_curr[0:3,0:3].t()
        transform : torch.Tensor = torch.cat([Rinv, torch.matmul(Rinv, -pose_curr[0:3,3]).unsqueeze(-1)], dim=1)

        bcurve_global : torch.Tensor = torch.ones(4, len(current_curve.control_points), dtype=transform.dtype, device=transform.device )
        for i in range(bcurve_global.shape[1]):
            bcurve_global[0,i]=current_curve.control_points[i].x
            bcurve_global[1,i]=current_curve.control_points[i].y
            bcurve_global[2,i]=current_curve.control_points[i].z
        bcurve_local = torch.matmul(transform, bcurve_global).T.unsqueeze(0)
        curve_msg : BezierCurve = C.toBezierCurveMsg(bcurve_local[0], std_msgs.msg.Header(frame_id=self.base_link_id, stamp=odom_msg.header.stamp))
        self.local_curve_pub.publish(curve_msg)
        dt : float = (float(current_curve.delta_t.sec) + float(current_curve.delta_t.nanosec)*1E-9)
        
        Msamp : torch.Tensor = mu.bezierM(self.tsamp, bcurve_local.shape[1]-1)
        Psamp : torch.Tensor = torch.matmul(Msamp[0], bcurve_local[0])
        arclengths : torch.Tensor = torch.zeros_like(self.tsamp[0])
        arclengths[1:]=torch.cumsum(torch.norm(Psamp[1:] - Psamp[:-1], p=2, dim=1), 0)

        _, _velocities = mu.bezierDerivative(bcurve_local, t=self.tsamp)
        velocities : torch.Tensor = _velocities[0]/dt
        speeds : torch.Tensor = torch.norm(velocities, p=2, dim=1)

        idx = torch.argmin(torch.norm(Psamp, p=2, dim=1))
        velocities = velocities[idx:]
        speeds = speeds[idx:]
        Psamp = Psamp[idx:]
        arclengths = arclengths[idx:]
        arclengths = arclengths - arclengths[0]

        current_speed : float = odom_msg.twist.twist.linear.x
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

        control_out : AckermannDriveStamped = AckermannDriveStamped(header=odom_msg.header)
        control_out.header.frame_id=self.base_link_id
        control_out.drive.steering_angle = torch.atan((self.twoL * torch.sin(alpha)) / ld).item()
        control_out.drive.speed=speeds[lookahead_index_vel].item()

        self.setpoint_pub.publish(control_out)
        self.lateral_error_pub.publish(Float64(data=torch.norm(Psamp[0], p=2).item()))

        


    
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