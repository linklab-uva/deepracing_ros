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

import os
import copy
import json
import numpy as np
from shapely import points

import ament_index_python
import rclpy
import rclpy.client
import rclpy.executors
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  
import rclpy.logging  
import rclpy.duration
import rclpy.parameter
import rclpy.time
import rclpy.timer
import deepracing_msgs.msg 
import deepracing_msgs.srv as deepracing_srvs   
import deepracing_ros.utils
import deepracing_ros.convert
import deepracing
import rclpy.executors 
import sensor_msgs_py.point_cloud2
import geometry_msgs.msg
import deepracing_models.math_utils as mu
import torch
from deepracing_rclpy.ghost_parameters import ghost_spawner
import functools

class GhostSpawner(rclpy.node.Node):
    def __init__(self, name="ghost_spawner"):
        super(GhostSpawner, self).__init__(name)
        self.raceline_helper : mu.RacelineHelper = None
        self.srv = None
        self.ghost_timer = None
        self.param_listener = ghost_spawner.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.ghost_pub : rclpy.publisher.Publisher = None
        self.frame_id : str | None = None
    def initialize(self, raceline_np : np.ndarray, frame_id : str):
        racelinepoints = torch.as_tensor(np.stack([raceline_np[k] for k in ["x", "y", "z"]], axis=1), dtype=torch.float64)#, device=torch.device("cuda:0"))
        racelinespeeds = self.params.timescale*torch.as_tensor(raceline_np["speed"]).type_as(racelinepoints)
        self.get_logger().info("Building raceline helper")
        self.raceline_helper = mu.RacelineHelper.from_closed_path(racelinepoints, racelinespeeds)
        self.get_logger().info("Built raceline helper")
        self.frame_id = frame_id
        self.ghost_pub : rclpy.publisher.Publisher = self.create_publisher(geometry_msgs.msg.PointStamped, "ghost_position", 1)
        self.srv = self.create_service(deepracing_srvs.SpawnGhost, 'spawn_ghost', self.spawn_ghost_cb)
        # tsamp = torch.linspace(0.0, self.params.prediction_horizon, steps=60).type_as(racelinepoints)
        # rsamp, pointssamp, velssamp, _ = self.raceline_helper(t=tsamp)
        # print(pointssamp)
        # print(torch.norm(velssamp, p=2.0, dim=-1))
        # print(rsamp)
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
        tsamp = torch.linspace(t0, t0 + self.params.prediction_horizon, steps=30).type_as(self.raceline_helper.__arclengths_in__)
        rsamp, pointssamp, velssamp, _ = self.raceline_helper(t=tsamp)
        controlpoints_fit, tswitch = mu.compositeBezierFit(tsamp[None] - tsamp[0], pointssamp[None], 4, Y_0=pointssamp[[0,]])
        pointstamped = geometry_msgs.msg.PointStamped()
        pointstamped.header.stamp = now.to_msg()
        pointstamped.header.frame_id = self.frame_id
        pointstamped.point.x = controlpoints_fit[0,0,0,0].item()
        pointstamped.point.y = controlpoints_fit[0,0,0,1].item()
        pointstamped.point.z = controlpoints_fit[0,0,0,2].item()
        self.ghost_pub.publish(pointstamped)
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