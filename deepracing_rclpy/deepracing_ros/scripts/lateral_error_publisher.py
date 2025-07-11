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

import numpy as np

import rclpy
import rclpy.qos
import rclpy.client
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  
import rclpy.logging  
import rclpy.time
import rclpy.timer
import deepracing_ros.convert as C
import deepracing_msgs.msg 
import deepracing_msgs.srv as deepracing_srvs   

import sensor_msgs_py.point_cloud2
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import deepracing_models.math_utils as mu
import torch
import functools
import tf2_ros
from scipy.spatial.transform import Rotation
import time
import threading


class LateralErrorPublisher(rclpy.node.Node):
    def __init__(self, name="lateral_error_publisher"):
        super(LateralErrorPublisher, self).__init__(name)
        self.odom_sub : rclpy.subscription.Subscription = self.create_subscription(nav_msgs.msg.Odometry, "odom", self.odom_cb, 1)
        self.raceline_helper : mu.RacelineHelper = None
        self.lateral_error_pub : rclpy.publisher.Publisher = None
        self.refpoint_pub : rclpy.publisher.Publisher = None 

        gpu_param = self.declare_parameter("gpu", value=-1)
        self.gpu : int = gpu_param.get_parameter_value().integer_value

        newton_iterations_param = self.declare_parameter("newton_iterations", value=-1)
        self.newton_iterations : int = newton_iterations_param.get_parameter_value().integer_value
    def odom_cb(self, odom : nav_msgs.msg.Odometry):
        if self.raceline_helper is None:
            return
        position = odom.pose.pose.position
        quaternion = odom.pose.pose.orientation
        position = torch.as_tensor([position.x, position.y, position.z]).type_as(self.raceline_helper.__arclengths_in__)
        rotmat = torch.as_tensor(Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()).type_as(self.raceline_helper.__arclengths_in__)
        
        if self.newton_iterations>0:
            _, (pintersect,), _, _ = self.raceline_helper.__curve_of_r__.y_axis_intersection_approximate(position[None], rotmat[None], newton_iterations=self.newton_iterations)
        else:
            rintersect = self.raceline_helper.__curve_of_r__.y_axis_intersection(position[None], rotmat[None])
            _, (pintersect,), _, _ = self.raceline_helper(r=rintersect)
        

        pclosest_local : torch.Tensor = ((pintersect - position)[None] @ rotmat)[0]

        pointstamped = geometry_msgs.msg.PointStamped(header=odom.header)
        pointstamped.header.frame_id=odom.child_frame_id
        pointstamped.point.x = pclosest_local[0].item()
        pointstamped.point.y = pclosest_local[1].item()
        pointstamped.point.z = pclosest_local[2].item()
        self.lateral_error_pub.publish(std_msgs.msg.Float64(data=pclosest_local[1].item()))
        self.refpoint_pub.publish(pointstamped)


    def initialize(self, raceline_np : np.ndarray, frame_id : str):
        racelinepoints = torch.as_tensor(np.stack([raceline_np[k] for k in ["x", "y", "z"]], axis=1), dtype=torch.float64)#, device=torch.device("cuda:0"))
        
        racelinespeeds = torch.as_tensor(raceline_np["speed"]).type_as(racelinepoints)
        self.get_logger().info("Building raceline helper")
        self.raceline_helper = mu.RacelineHelper.from_closed_path(racelinepoints, racelinespeeds, 1.5).to(tensor=racelinepoints)
        self.frame_id = frame_id

        if self.gpu>=0: self.raceline_helper = self.raceline_helper.cuda(self.gpu)
        if self.newton_iterations>0:
            quat = torch.as_tensor([0.0, 0.0, 0.0, 1.0], dtype=torch.float64)
            Rquery = torch.as_tensor(Rotation.from_quat((quat + 0.05*torch.rand_like(quat)).numpy()).as_matrix())[None].type_as(self.raceline_helper.__arclengths_in__)
            Pquery = torch.randn_like(Rquery[:,0])
            self.raceline_helper.__curve_of_r__.y_axis_intersection_approximate(Pquery, Rquery, newton_iterations=self.newton_iterations)

        self.lateral_error_pub = self.create_publisher(std_msgs.msg.Float64, "lateral_error", 1)
        self.refpoint_pub = self.create_publisher(geometry_msgs.msg.PointStamped, "reference_point", 1)
        self.get_logger().info("Built raceline helper")

        # print(pointssamp[0])
        



        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = LateralErrorPublisher()
    getline_client : rclpy.client.Client = node.create_client(deepracing_srvs.GetLine, "/get_line")
    getline_client.wait_for_service()
    getlinereq : deepracing_srvs.GetLine.Request = deepracing_srvs.GetLine.Request()
    getlinereq.key.data="raceline"
    success = False
    while not success:
        future = getline_client.call_async(getlinereq)
        rclpy.spin_until_future_complete(node, future)
        getlineresponse : deepracing_srvs.GetLine.Response = future.result()
        if getlineresponse.return_code==deepracing_srvs.GetLine.Response.SUCCESS:
            success = True
            node.get_logger().info("Successfully got the raceline")
        else:
            node.get_logger().error("Unable to get the raceline. Error code: %d." % (getlineresponse.return_code,))
            # exit(-1)
    racelinenp = sensor_msgs_py.point_cloud2.read_points(getlineresponse.line)
    node.initialize(racelinenp, getlineresponse.line.header.frame_id)
    rclpy.spin(node)#, rclpy.executors.MultiThreadedExecutor(num_threads=3))

if __name__ == '__main__':
    main()