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
from deepracing_msgs.msg import BezierCurve, TimestampedPacketSessionData, CompositeBezierCurve
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
import deepracing_models.math_utils.bezier as bezier
import tf2_ros
import rclpy.time
import rclpy.duration
import threading 
import deepracing_ros.convert as C
import std_msgs.msg
import rclpy.qos

class BezierCurvePurePursuit(Node):
    def __init__(self,):
        super(BezierCurvePurePursuit,self).__init__('bezier_pure_pursuit')
        gpu_param : Parameter = self.declare_parameter("gpu", value=-1)
        gpu : int = gpu_param.get_parameter_value().integer_value

        if torch.backends.cuda.is_built() and gpu>=0:
            self.get_logger().info("Running on GPU %d" %(gpu,))
            self.device : torch.device = torch.device("cuda:%d" % (gpu,))
        else:
            self.get_logger().info("Running on the CPU")
            self.device : torch.device = torch.device("cpu")

        self.tsamp : torch.Tensor = torch.linspace(0.0, 1.0, 300, dtype=torch.float64, device=self.device).unsqueeze(0)
        self.matrix_factories : dict[int, bezier.BezierMatrixFactory] = dict()
        for i in range(1, 7):
            self.matrix_factories[i] = (bezier.BezierMatrixFactory(i).to(tensor=self.tsamp))
            self.matrix_factories[i](self.tsamp)
            #torch.compile, fullgraph=True, mode="max-autotune-no-cudagraphs"
        # self.cbc_evaluators : dict[int, mu.CBCEvaluator] = dict()
        # fake_nseg = 4
        # fake_tswitch = torch.linspace(0.0, 6.0, steps=fake_nseg+1).type_as(self.tsamp)
        # fake_tstart = fake_tswitch[:-1]
        # fake_deltat = fake_tswitch[1:] - fake_tstart
        # for i in range(1, 7):
        #     fake_curve = torch.randn([fake_nseg, i+1, 3]).type_as(self.tsamp)
        #     self.cbc_evaluators[i] = torch.compile(mu.CBCEvaluator(i).to(tensor=self.tsamp), dynamic=True, fullgraph=True, mode="max-autotune-no-cudagraphs")
        #     self.cbc_evaluators[i](fake_tstart[None], fake_deltat[None], fake_curve[None], fake_tswitch[-1]*self.tsamp)
            
        self.setpoint_pub : Publisher = self.create_publisher(AckermannDriveStamped, "ctrl_cmd", 1)# rclpy.qos.qos_profile_sensor_data)
        self.lateral_error_pub : Publisher = self.create_publisher(Float64, "lateral_error", 1)
        # self.local_curve_pub : Publisher = self.create_publisher(BezierCurve, "local_bezier_curves", 1)
        # self.curve_sub : Subscription = self.create_subscription(BezierCurve, "beziercurves_in", self.curveCB, 1)
        rate_param : Parameter = self.declare_parameter("rate", value=100.0)

        self.curve_sub : Subscription = self.create_subscription(CompositeBezierCurve, "compositebeziercurves_in", self.curveCB, 1)
        self.timer = self.create_timer(1.0/rate_param.get_parameter_value().double_value, self.timerCB)

        self.current_odom = Odometry()
        self.odom_sub : Subscription = self.create_subscription(Odometry, "odom", self.odomCB, 1)
        self.session_sub : Subscription = self.create_subscription(TimestampedPacketSessionData, "/session_data", self.sessionCB, 1)
        self.current_curve_msg : CompositeBezierCurve = None
        self.current_curve_mutex : threading.Semaphore = threading.Semaphore()
        self.current_odom_mutex : threading.Semaphore = threading.Semaphore()

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



        self.twoL : torch.Tensor = torch.as_tensor(2.0*wheelbase, dtype=self.tsamp.dtype, device=self.tsamp.device)

        self.player_car_index : int = 0



    def sessionCB(self, session_msg : TimestampedPacketSessionData):
        self.player_car_index = session_msg.udp_packet.header.player_car_index

    def curveCB(self, curve_msg : CompositeBezierCurve):
        if not self.current_curve_mutex.acquire(timeout=0.5):
            self.get_logger().error("Unable to acquire current_curve_mutex")
            return
        self.current_curve_msg = curve_msg
        self.current_curve_mutex.release()

    def odomCB(self, odom : Odometry):
        if not self.current_odom_mutex.acquire(timeout=0.5):
            self.get_logger().error("Unable to acquire current_odom_mutex")
            return
        self.current_odom = odom
        self.current_odom_mutex.release()

    def timerCB(self):
        if self.current_curve_msg is None:
            self.get_logger().debug("No bezier curve received yet")
            return
        
        
        if not self.current_curve_mutex.acquire(timeout=0.1):
            self.get_logger().error("Unable to acquire current_curve_mutex")
            return
        # curveheader = deepcopy(self.current_curve_msg.header)
        delta_t, control_points_global = C.fromCompositeBezierCurveMsg(self.current_curve_msg, dtype=self.tsamp.dtype, device=self.tsamp.device)
        self.current_curve_mutex.release()

        if not self.current_odom_mutex.acquire(timeout=0.1):
            self.get_logger().error("Unable to acquire current_odom_mutex")
            return
        # poseheader = deepcopy(self.current_odom.header)
        # posechildframe = deepcopy(self.current_odom.child_frame_id)
        posemat = C.poseMsgToTorch(self.current_odom.pose.pose, dtype=self.tsamp.dtype, device=self.tsamp.device)
        self.current_odom_mutex.release()


        pose_R = posemat[0:3,0:3]
        pose_T = posemat[0:3,3]
        Rinv = pose_R.T
        Tinv = -(Rinv@pose_T[:,None])[:,0]
        control_points = (Rinv[None,None]@control_points_global[...,None])[...,0] + Tinv[None,None]


        # print(posemat)
        # if not (curveheader.frame_id==posechildframe)


        tend = torch.cumsum(delta_t, 0)
        tstart = tend - delta_t[0]
        tsamp = tend[-1]*self.tsamp
        kbezier = int(control_points.shape[1]) - 1
        control_points_deriv = kbezier*torch.diff(control_points, dim=1)/delta_t[:,None,None]
        # if kbezier not in self.matrix_factories:
        #     self.matrix_factories[kbezier] = bezier.BezierMatrixFactory(kbezier).to(tensor=self.tsamp)
        # if (kbezier-1) not in self.matrix_factories:
        #     self.matrix_factories[kbezier-1] = bezier.BezierMatrixFactory(kbezier-1).to(tensor=self.tsamp)
        # (Psamp,), idxbuckets = self.cbc_evaluators[kbezier](tstart[None], delta_t[None], control_points[None], tsamp)
        (Psamp,), idxbuckets = mu.compositeBezierEval(tstart[None], delta_t[None], control_points[None], tsamp, self.matrix_factories[kbezier])
        # (velocities,), _ = self.cbc_evaluators[kbezier-1](tstart[None], delta_t[None], control_points_deriv[None], tsamp, idxbuckets=idxbuckets)
        (velocities,), _ = mu.compositeBezierEval(tstart[None], delta_t[None], control_points_deriv[None], tsamp, self.matrix_factories[kbezier-1], idxbuckets=idxbuckets)


        arclengths : torch.Tensor = torch.zeros_like(self.tsamp[0])
        arclengths[1:]=torch.cumsum(torch.norm(Psamp[1:] - Psamp[:-1], p=2, dim=1), 0)

        speeds : torch.Tensor = torch.norm(velocities, p=2, dim=1)

        idx = torch.argmin(torch.norm(Psamp, p=2, dim=1))
        velocities = velocities[idx:]
        speeds = speeds[idx:]
        Psamp = Psamp[idx:]
        arclengths = arclengths[idx:]
        arclengths = arclengths - arclengths[0]

        current_speed : float = float(self.current_odom.twist.twist.linear.x)
        lookahead_distance = max(self.lookahead_gain*current_speed, 10.0)
        # if current_speed>55.0:
        #     lookahead_distance_vel = self.velocity_lookahead_gain*current_speed
        # else:
        #     lookahead_distance_vel=0.00
        lookahead_distance_vel = self.velocity_lookahead_gain*current_speed

        lookahead_index = torch.argmin(torch.abs(arclengths-lookahead_distance))
        lookahead_index_vel = torch.argmin(torch.abs(arclengths-lookahead_distance_vel))

        lookaheadVector = Psamp[lookahead_index]
        ld = torch.norm(lookaheadVector, p=2)
        lookaheadDirection = lookaheadVector/ld
        alpha = torch.atan2(lookaheadDirection[1],lookaheadDirection[0])

        control_out : AckermannDriveStamped = AckermannDriveStamped()
        control_out.header.frame_id=self.base_link_id
        control_out.header.stamp = self.get_clock().now().to_msg()
        control_out.drive.steering_angle = torch.atan((self.twoL * torch.sin(alpha)) / ld).item()
        control_out.drive.speed=speeds[lookahead_index_vel].item()

        self.setpoint_pub.publish(control_out)
        # self.lateral_error_pub.publish(Float64(data=torch.norm(Psamp[0], p=2).item()))

        


    
def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BezierCurvePurePursuit()
    num_threads_param : rclpy.Parameter = node.declare_parameter("num_threads", 0)
    num_threads : int = num_threads_param.get_parameter_value().integer_value
    # if num_threads<=0:
    #     node.get_logger().info("Spinning with number of CPU cores")
    #     spinner : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor(None)
    # else:
    #     node.get_logger().info("Spinning with %d threads" % (num_threads,))
    #     spinner : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor(num_threads)
    # spinner.add_node(node)
    # try:
    #     spinner.spin()
    # except KeyboardInterrupt:
    #     pass
    rclpy.spin(node)

if __name__ == '__main__':
    main()