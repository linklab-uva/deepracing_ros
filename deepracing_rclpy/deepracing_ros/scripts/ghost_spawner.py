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
from deepracing_rclpy.ghost_parameters import ghost_spawner
import functools
import tf2_ros
from scipy.spatial.transform import Rotation
import time
import threading


class GhostSpawner(rclpy.node.Node):
    def __init__(self, name="ghost_spawner"):
        super(GhostSpawner, self).__init__(name)
        self.raceline_helper : mu.RacelineHelper = None
        self.srv = None
        self.ghost_timer = None
        self.ghost_position_pub : rclpy.publisher.Publisher = None
        self.ghost_prediction_pub : rclpy.publisher.Publisher = None
        self.ghost_t_pub : rclpy.publisher.Publisher = None
        self.ghost_r_pub : rclpy.publisher.Publisher = None
        self.frame_id : str | None = None
        self.tdelta : torch.Tensor | None = None
        self.param_listener = ghost_spawner.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.current_ego_odom : nav_msgs.msg.Odometry | None = None
        self.current_ego_odom_mutex = threading.Semaphore()
        self.ego_odom_sub : rclpy.subscription.Subscription = self.create_subscription(nav_msgs.msg.Odometry, "ego_odom", self.odom_cb, 1)
        self.tf2_broadcaster : tf2_ros.TransformBroadcaster = tf2_ros.TransformBroadcaster(self)
    def odom_cb(self, odom : nav_msgs.msg.Odometry):
        if not self.current_ego_odom_mutex.acquire(timeout=2E-1):
            raise ValueError("Unable to acquire self.current_ego_odom_mutex")
        self.current_ego_odom = odom
        self.current_ego_odom_mutex.release()
    def initialize(self, raceline_np : np.ndarray, frame_id : str):
        racelinepoints = torch.as_tensor(np.stack([raceline_np[k] for k in ["x", "y", "z"]], axis=1), dtype=torch.float64)#, device=torch.device("cuda:0"))
        if self.params.gpu>=0: racelinepoints = racelinepoints.cuda(self.params.gpu)
        racelinespeeds = self.params.timescale*torch.as_tensor(raceline_np["speed"]).type_as(racelinepoints)
        self.get_logger().info("Building raceline helper")
        raceline_helper = mu.RacelineHelper.from_closed_path(racelinepoints, racelinespeeds, 1.0).to(tensor=racelinepoints)#.float()
        self.get_logger().info("Built raceline helper")
        self.tdelta = torch.linspace(0.0, self.params.prediction_horizon, steps=30).type_as(raceline_helper.__arclengths_in__)


        self.get_logger().info("Compiling raceline helper")
        with torch.no_grad():
            self.raceline_helper = torch.compile(raceline_helper, mode="max-autotune-no-cudagraphs", fullgraph=True)
            self.raceline_helper.__t_of_r__.compile(mode="max-autotune-no-cudagraphs", fullgraph=True, dynamic=True)
            dummyt = self.tdelta + self.raceline_helper.__r_of_t__.xstart_vec[torch.randint(0, high=self.raceline_helper.__r_of_t__.xstart_vec.shape[0], size=[1,]).item()].item()
            dummyr, _, _, _ = self.raceline_helper(t=dummyt)
            self.raceline_helper(r=dummyr)

            rtest : torch.Tensor = dummyr.clone()#self.raceline_helper.__arclengths_in__[-1]*(torch.rand(6).type_as(self.raceline_helper.__arclengths_in__))
            tequavalent = self.raceline_helper.t_of_r(rtest)
            self.raceline_helper.__r_of_t__(tequavalent)
            # tequavalent2 = self.raceline_helper.t_of_r(rtest[[0,]])
            # self.raceline_helper.__r_of_t__(tequavalent2)
            rback : torch.Tensor = self.raceline_helper.__r_of_t__(tequavalent)[0].squeeze(-1)
            self.get_logger().info("Compiled raceline helper")
            self.get_logger().info("rtest: %s" % (str(rtest),))
            self.get_logger().info("rback: %s" % (str(rback),))
            self.get_logger().info("dummyt: %s" % (str(dummyt),))
            self.get_logger().info("tequavalent: %s" % (str(tequavalent),))
            comptimes = torch.empty(20, dtype=torch.float64)
            for i in range(comptimes.shape[0]):
                rtest = self.raceline_helper.__t_of_r__.arclengths[-1]*torch.rand_like(rtest)
                tick = time.time()
                tequavalent = self.raceline_helper.t_of_r(rtest)
                tock = time.time()
                comptimes[i] = (tock - tick)
            self.get_logger().info("comptimes: %s" % (str(1000.0*comptimes),))





        self.frame_id = frame_id
        self.ghost_position_pub : rclpy.publisher.Publisher = self.create_publisher(geometry_msgs.msg.PointStamped, "target_position", rclpy.qos.qos_profile_sensor_data)
        self.ghost_prediction_pub : rclpy.publisher.Publisher = self.create_publisher(deepracing_msgs.msg.CompositeBezierCurve, "target_prediction", rclpy.qos.qos_profile_sensor_data)
        self.ghost_polygon_pub : rclpy.publisher.Publisher = self.create_publisher(geometry_msgs.msg.PolygonStamped, "polygon", rclpy.qos.qos_profile_sensor_data)
        self.ghost_t_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "target_time", rclpy.qos.qos_profile_sensor_data)
        self.ghost_r_pub : rclpy.publisher.Publisher = self.create_publisher(std_msgs.msg.Float64, "target_arclength", rclpy.qos.qos_profile_sensor_data)
        self.srv = self.create_service(deepracing_srvs.SpawnGhost, 'spawn_ghost', self.spawn_ghost_cb)

    def spawn_ghost_cb(self, request : deepracing_srvs.SpawnGhost.Request, response : deepracing_srvs.SpawnGhost.Response):
        response.return_code=deepracing_srvs.SpawnGhost.Response.UNKNOWN
        if self.ghost_timer is not None:
            self.destroy_timer(self.ghost_timer)
        if request.relative_to_ego:
            if (self.current_ego_odom is None):
                response.return_code=deepracing_srvs.SpawnGhost.Response.NO_EGO_ODOM
                response.message="Ego odom not yet received on topic: %s" % (self.ego_odom_sub.topic)
                return response
            if not self.current_ego_odom_mutex.acquire(timeout=2E-1):
                response.return_code=deepracing_srvs.SpawnGhost.Response.NO_EGO_ODOM
                response.message="Could not acquire odom mutex"
                return response
            odom_position = self.current_ego_odom.pose.pose.position
            current_ego_position = torch.as_tensor([odom_position.x, odom_position.y, odom_position.z])
            self.current_ego_odom_mutex.release()
            current_ego_position = current_ego_position.type_as(self.raceline_helper.__arclengths_in__)
            rclosest, _, _, _ = self.raceline_helper.closest_point_approximate(current_ego_position[None], newton_iterations=4)
            tclosest = self.raceline_helper.t_of_r(rclosest).item()
            tstart_rl = tclosest + request.raceline_tstart
        else:
            tstart_rl = request.raceline_tstart
        self.get_logger().info("Spawning ghost starting at time %f" % (tstart_rl,))
        now = self.get_clock().now()
        self.ghost_timer : rclpy.timer.Timer = self.create_timer(
            1.0/request.frequency, functools.partial(self.ghost_timer_cb,
             tstart_rl=tstart_rl, tstart_global=now, constraint_level=request.constraint_level, kbezier=request.kbezier, num_segments=request.num_segments))
        response.message="Yay"
        return response

    def ghost_timer_cb(self, *, tstart_rl : float, tstart_global : rclpy.time.Time, constraint_level : int, kbezier : int, num_segments : int):
        now = self.get_clock().now()
        delta = now - tstart_global
        delta_float = 1E-9*float(delta.nanoseconds)
        t0 = (tstart_rl + delta_float)%self.raceline_helper.__r_of_t__.xend_vec[-1].item()
        rsamp, pointssamp, velssamp, _ = self.raceline_helper(t=self.tdelta+t0)
        (controlpoints_fit,), (tswitch,) = mu.compositeBezierFit(self.tdelta[None], pointssamp[None], num_segments, Y_0=pointssamp[[0,]], dYdT_0=velssamp[[0,]], constraint_level=constraint_level, kbezier=kbezier)
        deltat = tswitch[1:] - tswitch[:-1]

        p0 = controlpoints_fit[0,0]
        tau0 = controlpoints_fit[0,1] - p0
        tau0 = tau0/torch.linalg.vector_norm(tau0)

        up = torch.zeros_like(tau0)
        up[-1] = 1.0

        nu0 = torch.linalg.cross(up, tau0)

        zvec = torch.linalg.cross(tau0, nu0)

        rotmat = torch.stack([tau0, nu0, zvec], dim=-1)

        rot = Rotation.from_matrix(rotmat.cpu().numpy())
        quat = rot.as_quat()

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.frame_id
        transform.transform.rotation = geometry_msgs.msg.Quaternion(x=quat[0].item(), y=quat[1].item(), z=quat[2].item(), w=quat[3].item())
        transform.transform.translation = geometry_msgs.msg.Vector3(x=p0[0].item(), y=p0[1].item(), z=p0[2].item())
        transform.child_frame_id=self.params.tf_frame
        
        cbc_msg = C.toCompositeBezierCurveMsg(deltat, controlpoints_fit, header=transform.header)

        point_msg = geometry_msgs.msg.PointStamped(header=cbc_msg.header)
        point_msg.point=cbc_msg.control_points_flat[0]
        
        car_length, car_width = 5.2, 2.0
        polygon_msg = geometry_msgs.msg.PolygonStamped()
        polygon_msg.header.stamp = transform.header.stamp
        polygon_msg.header.frame_id = transform.child_frame_id
        polygon_msg.polygon.points.append(geometry_msgs.msg.Point32(x=-0.5*car_length, y=-0.5*car_width))
        polygon_msg.polygon.points.append(geometry_msgs.msg.Point32(x=0.5*car_length, y=-0.5*car_width))
        polygon_msg.polygon.points.append(geometry_msgs.msg.Point32(x=0.5*car_length, y=0.5*car_width))
        polygon_msg.polygon.points.append(geometry_msgs.msg.Point32(x=-0.5*car_length, y=0.5*car_width))
        self.tf2_broadcaster.sendTransform(transform)
        self.ghost_t_pub.publish(std_msgs.msg.Float64(data=t0))
        self.ghost_r_pub.publish(std_msgs.msg.Float64(data=rsamp[0].item()))        
        self.ghost_position_pub.publish(point_msg)
        self.ghost_prediction_pub.publish(cbc_msg)
        self.ghost_polygon_pub.publish(polygon_msg)
        
        # print(pointssamp[0])
        



        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = GhostSpawner()
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