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
import rclpy.logging  
import rclpy.time
import rclpy.timer
import deepracing_msgs.msg 
import deepracing_msgs.srv as deepracing_srvs   

import sensor_msgs_py.point_cloud2
import geometry_msgs.msg
import deepracing_models.math_utils as mu
import torch
from deepracing_rclpy.ghost_parameters import ghost_spawner
import functools
import tf2_ros
from scipy.spatial.transform import Rotation


class GhostSpawner(rclpy.node.Node):
    def __init__(self, name="ghost_spawner"):
        super(GhostSpawner, self).__init__(name)
        self.raceline_helper : mu.RacelineHelper = None
        self.srv = None
        self.ghost_timer = None
        self.ghost_position_pub : rclpy.publisher.Publisher = None
        self.ghost_prediction_pub : rclpy.publisher.Publisher = None
        self.frame_id : str | None = None
        self.param_listener = ghost_spawner.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.tf2_broadcaster : tf2_ros.TransformBroadcaster = tf2_ros.TransformBroadcaster(self)

    def initialize(self, raceline_np : np.ndarray, frame_id : str):
        racelinepoints = torch.as_tensor(np.stack([raceline_np[k] for k in ["x", "y", "z"]], axis=1), dtype=torch.float64)#, device=torch.device("cuda:0"))
        racelinespeeds = self.params.timescale*torch.as_tensor(raceline_np["speed"]).type_as(racelinepoints)
        self.get_logger().info("Building raceline helper")
        self.raceline_helper = mu.RacelineHelper.from_closed_path(racelinepoints, racelinespeeds, 1.0)
        self.get_logger().info("Built raceline helper")
        self.frame_id = frame_id
        self.ghost_position_pub : rclpy.publisher.Publisher = self.create_publisher(geometry_msgs.msg.PointStamped, "target_position", rclpy.qos.qos_profile_sensor_data)
        self.ghost_prediction_pub : rclpy.publisher.Publisher = self.create_publisher(deepracing_msgs.msg.CompositeBezierCurve, "target_prediction", rclpy.qos.qos_profile_sensor_data)
        self.srv = self.create_service(deepracing_srvs.SpawnGhost, 'spawn_ghost', self.spawn_ghost_cb)

    def spawn_ghost_cb(self, request : deepracing_srvs.SpawnGhost.Request, response : deepracing_srvs.SpawnGhost.Response):
        self.get_logger().info("Spawning ghost starting at time %f" % (request.raceline_tstart,))
        response.return_code=deepracing_srvs.SpawnGhost.Response.UNKNOWN
        if self.ghost_timer is not None:
            self.destroy_timer(self.ghost_timer)
        now = self.get_clock().now()
        self.ghost_timer : rclpy.timer.Timer = self.create_timer(1.0/request.frequency, functools.partial(self.ghost_timer_cb, request.raceline_tstart, now))
        return response

    def ghost_timer_cb(self, tstart_rl : float, tstart_global : rclpy.time.Time):
        now = self.get_clock().now()
        delta = now - tstart_global
        delta_float = 1E-9*float(delta.nanoseconds)
        t0 = (tstart_rl + delta_float)%self.raceline_helper.__r_of_t__.xend_vec[-1].item()
        tdelta = torch.linspace(0.0, self.params.prediction_horizon, steps=30).type_as(self.raceline_helper.__arclengths_in__)
        rsamp, pointssamp, velssamp, _ = self.raceline_helper(t=tdelta+t0)
        (controlpoints_fit,), (tswitch,) = mu.compositeBezierFit(tdelta[None], pointssamp[None], 4, Y_0=pointssamp[[0,]], constraint_level=2)
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
        
        cbc_msg = deepracing_msgs.msg.CompositeBezierCurve()
        cbc_msg.header = transform.header
        cbc_msg.delta_t = deltat.cpu().numpy()#.tolist() 
        cbc_msg.segments = controlpoints_fit.shape[0]
        cbc_msg.order = controlpoints_fit.shape[1]-1
        cbc_msg.two_d=False
        controlpoints_flat = controlpoints_fit.view(-1, 3)
        for i in range(controlpoints_flat.shape[0]):
            point = geometry_msgs.msg.Point()
            point.x = controlpoints_flat[i,0].item()
            point.y = controlpoints_flat[i,1].item()
            point.z = controlpoints_flat[i,2].item()
            cbc_msg.control_points_flat.append(point)

        point_msg = geometry_msgs.msg.PointStamped()
        point_msg.header=cbc_msg.header
        point_msg.point=cbc_msg.control_points_flat[0]
        
        self.tf2_broadcaster.sendTransform(transform)
        self.ghost_position_pub.publish(point_msg)
        self.ghost_prediction_pub.publish(cbc_msg)
        
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
            exit(-1)
    racelinenp = sensor_msgs_py.point_cloud2.read_points(getlineresponse.line)
    node.initialize(racelinenp, getlineresponse.line.header.frame_id)
    rclpy.spin(node)#, rclpy.executors.MultiThreadedExecutor(num_threads=3))

if __name__ == '__main__':
    main()