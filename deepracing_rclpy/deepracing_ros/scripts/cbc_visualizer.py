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
from regex import D

import ament_index_python
import rclpy
import rclpy.qos
import rclpy.duration
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  
import deepracing_msgs.msg 
import deepracing_ros.convert as C
import std_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import deepracing_models.math_utils as mu
import torch
from deepracing_rclpy.ghost_parameters import ghost_spawner
import matplotlib.pyplot as plt
import matplotlib, matplotlib.colors, matplotlib.cm
from deepracing_rclpy.cbc_viz import cbc_visualizer


class CBCVisualizer(rclpy.node.Node):
    def __init__(self, name="cbc_viz"):
        super(CBCVisualizer, self).__init__(name)
        self.cbc_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.CompositeBezierCurve, "curve_in", self.cbc_cb, rclpy.qos.qos_profile_sensor_data)
        self.marker_pub : rclpy.publisher.Publisher = self.create_publisher(visualization_msgs.msg.MarkerArray, "marker_out", rclpy.qos.qos_profile_sensor_data)
        self.matrix_factories = dict()
        prop_cycle = plt.rcParams['axes.prop_cycle']
        self.colorcycle = list(prop_cycle.by_key()['color'])
        self.param_listener = cbc_visualizer.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.cm = matplotlib.cm.get_cmap(name=self.params.color_map)

    def cbc_cb(self, cbc_msg : deepracing_msgs.msg.CompositeBezierCurve):

        delta_t, control_points = C.fromCompositeBezierCurveMsg(cbc_msg, dtype=torch.float64)
        tstart = torch.cumsum(delta_t, 0) - delta_t[0]
        tsamp = torch.linspace(0.0, tstart[-1] + delta_t[-1], steps=self.params.num_sample_points).type_as(control_points)

        if cbc_msg.order not in self.matrix_factories:
            self.matrix_factories[cbc_msg.order] = mu.BezierMatrixFactory(cbc_msg.order)
        if (cbc_msg.order-1) not in self.matrix_factories:
            self.matrix_factories[cbc_msg.order-1] = mu.BezierMatrixFactory(cbc_msg.order-1)

        points_marker = visualization_msgs.msg.Marker()
        points_marker.header=cbc_msg.header
        points_marker.action=visualization_msgs.msg.Marker.ADD
        points_marker.type=visualization_msgs.msg.Marker.POINTS
        points_marker.ns=self.params.namespace
        points_marker.id=1
        points_marker.lifetime=rclpy.duration.Duration(seconds=0, nanoseconds=int(.5E9)).to_msg()
        points_marker.pose=geometry_msgs.msg.Pose()
        points_marker.scale=geometry_msgs.msg.Vector3(x=self.params.control_point_scale, y=self.params.control_point_scale, z=self.params.control_point_scale)
        points_marker.colors=[]
        points_marker.points=[]
        pps = cbc_msg.order+1
        rgb0=matplotlib.colors.to_rgb(self.colorcycle[0])
        for i in range(pps-1):
            points_marker.colors.append(std_msgs.msg.ColorRGBA(r=rgb0[0], g=rgb0[1], b=rgb0[2], a=1.0))
            points_marker.points.append(cbc_msg.control_points_flat[i])
        for segment_idx in range(1, cbc_msg.segments):
            rgbprev = np.asarray(matplotlib.colors.to_rgb(self.colorcycle[(segment_idx-1)%len(self.colorcycle)]))
            rgbcurr = np.asarray(matplotlib.colors.to_rgb(self.colorcycle[segment_idx%len(self.colorcycle)]))
            rbgmixed = 0.5*(rgbprev+rgbcurr)
            i = segment_idx*pps
            points_marker.colors.append(std_msgs.msg.ColorRGBA(r=rbgmixed[0], g=rbgmixed[1], b=rbgmixed[2], a=1.0))
            points_marker.points.append(cbc_msg.control_points_flat[i])
            for point_idx in range(1, pps):
                i = segment_idx*pps + point_idx
                points_marker.colors.append(std_msgs.msg.ColorRGBA(r=rgbcurr[0], g=rgbcurr[1], b=rgbcurr[2], a=1.0))
                points_marker.points.append(cbc_msg.control_points_flat[i])

        (curve_points_samp,), idxbuckets = mu.compositeBezierEval(tstart[None], delta_t[None], control_points[None], tsamp[None], self.matrix_factories[cbc_msg.order])
        control_points_deriv = cbc_msg.order*torch.diff(control_points, dim=-2)/delta_t[:,None,None]
        (curve_vels_samp,), _ = mu.compositeBezierEval(tstart[None], delta_t[None], control_points_deriv[None], tsamp[None], self.matrix_factories[cbc_msg.order-1], idxbuckets=idxbuckets)
        curve_speeds_samp = torch.norm(curve_vels_samp, p=2.0, dim=-1)

        curve_marker = visualization_msgs.msg.Marker()
        curve_marker.header=cbc_msg.header
        curve_marker.action=visualization_msgs.msg.Marker.ADD
        curve_marker.type=visualization_msgs.msg.Marker.LINE_STRIP
        curve_marker.ns=points_marker.ns
        curve_marker.id=2
        # curve_marker.lifetime=points_marker.lifetime
        curve_marker.scale=geometry_msgs.msg.Vector3(x=self.params.curve_scale)
        curve_marker.color=std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        speed_factor_delta = self.params.speed_max-self.params.speed_min
        for i in range(curve_speeds_samp.shape[0]):
            speed_factor = torch.clip((curve_speeds_samp[i] - self.params.speed_min)/speed_factor_delta, min=0.0, max=1.0)
            rgb = matplotlib.colors.to_rgb(self.cm(speed_factor.item()))
            curve_marker.colors.append(std_msgs.msg.ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0))

        curve_marker.pose=geometry_msgs.msg.Pose()
        curve_marker.points=[
            geometry_msgs.msg.Point(x=curve_points_samp[i,0].item(),
                                    y=curve_points_samp[i,1].item(),
                                    z=0.0 if cbc_msg.two_d else curve_points_samp[i,2].item())
        for i in range(curve_points_samp.shape[0])]
        
        arrow_marker = visualization_msgs.msg.Marker()
        arrow_marker.header=cbc_msg.header
        arrow_marker.action=visualization_msgs.msg.Marker.ADD
        arrow_marker.type=visualization_msgs.msg.Marker.ARROW
        arrow_marker.color=points_marker.colors[0]
        arrow_marker.ns=points_marker.ns
        arrow_marker.id=3
        arrow_marker.lifetime=points_marker.lifetime
        arrow_marker.points=[cbc_msg.control_points_flat[0], cbc_msg.control_points_flat[1]]
        arrow_marker.scale=geometry_msgs.msg.Vector3(x=self.params.arrow_scale, y=1.5*self.params.arrow_scale)

        self.marker_pub.publish(visualization_msgs.msg.MarkerArray(markers=[points_marker, curve_marker, arrow_marker]))
        
        # print(pointssamp[0])
        



        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = CBCVisualizer()
    rclpy.spin(node)#, rclpy.executors.MultiThreadedExecutor(num_threads=3))

if __name__ == '__main__':
    main()