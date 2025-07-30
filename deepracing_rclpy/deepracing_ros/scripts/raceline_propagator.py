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


import threading
from turtle import pos
import rclpy
import rclpy.qos
import rclpy.client
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  

import deepracing_ros.convert as C
import deepracing_msgs.msg 
import deepracing_msgs.srv as deepracing_srvs   
import rclpy
import sensor_msgs_py.point_cloud2
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import deepracing_models.math_utils as mu
import numpy as np
import torch
import time
from scipy.spatial.transform import Rotation

class RacelinePropagator(rclpy.node.Node):
    STATE_PARAMETER_NAME="state"
    NSEGMENTS_PARAMETER_NAME="Nsegments"
    GPU_PARAMETER_NAME="gpu"
    TIMESCALE_PARAMETER_NAME="timescale"
    PREDICTION_HORIZON_PARAMETER_NAME="prediction_horizon"
    def __init__(self, name="raceline_propagator"):
        super(RacelinePropagator, self).__init__(name)
        self.raceline_helper : mu.RacelineHelper = None
        self.declare_parameter(RacelinePropagator.STATE_PARAMETER_NAME, value="CREATED")
        self.declare_parameter(RacelinePropagator.GPU_PARAMETER_NAME, value=-1)
        self.declare_parameter(RacelinePropagator.NSEGMENTS_PARAMETER_NAME, value=4)
        self.declare_parameter(RacelinePropagator.TIMESCALE_PARAMETER_NAME, value=1.0)
        self.declare_parameter(RacelinePropagator.PREDICTION_HORIZON_PARAMETER_NAME, value=7.0)
        self.prediction_pub = self.create_publisher(deepracing_msgs.msg.CompositeBezierCurve, "target_predictions", rclpy.qos.qos_profile_sensor_data)
        self.odom_sub : rclpy.subscription.Subscription = self.create_subscription(nav_msgs.msg.Odometry, "target_odom", self.odom_cb, 1)
    def odom_cb(self, odom : nav_msgs.msg.Odometry):
        state = self.get_parameter(RacelinePropagator.STATE_PARAMETER_NAME).get_parameter_value().string_value
        if state != "PREDICTING":
            self.get_logger().warn("Received odometry while not in PREDICTING state. Current state: %s" % (state,))
            return
        # self.get_logger().info("Received odometry")
        pos_msg : geometry_msgs.msg.Point = odom.pose.pose.position
        vel_msg : geometry_msgs.msg.Vector3 = odom.twist.twist.linear
        quat_msg : geometry_msgs.msg.Quaternion = odom.pose.pose.orientation

        quat = np.asarray([0.0, 0.0, quat_msg.z, quat_msg.w], dtype=np.float64)
        rot = Rotation.from_quat(quat)
        vel_local = np.asarray([vel_msg.x, vel_msg.y, 0.0], dtype=np.float64)   


        vel = torch.as_tensor(rot.apply(vel_local)[:2]).type_as(self.raceline_helper.__arclengths_in__)
        pos = torch.as_tensor([pos_msg.x, pos_msg.y,]).type_as(vel)

        closest_r, _, _, _ = self.raceline_helper.closest_point_approximate(pos[None], newton_iterations=4)

        closest_t = self.raceline_helper.t_of_r(closest_r)[0]
        
        
        # prediction_horizon = self.get_parameter(RacelinePropagator.PREDICTION_HORIZON_PARAMETER_NAME).get_parameter_value().double_value
        # t_forward = torch.linspace(closest_t, closest_t + prediction_horizon, steps=30).type_as(vel)
        t_forward = self.tdelta + closest_t
        r_forward, rl_points, rl_vels, _ = self.raceline_helper(t=t_forward)

        #t_fit = self.tdelta

        Nsegments = self.get_parameter(RacelinePropagator.NSEGMENTS_PARAMETER_NAME).get_parameter_value().integer_value
        control_points, tswitch = mu.compositeBezierFit(self.tdelta, rl_points, Nsegments, Y_0=pos, dYdT_0=vel, constraint_level=2, kbezier=3)

        delta_t = torch.diff(tswitch, dim=0)
      
    
        cbc_msg = C.toCompositeBezierCurveMsg(delta_t, control_points, header=odom.header)
        # print(cbc_msg)
        self.prediction_pub.publish(cbc_msg)


    def initialize(self, raceline_np : np.ndarray):
        stateparam = rclpy.Parameter(RacelinePropagator.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "INITIALIZING")
        self.set_parameters([stateparam,])
        racelinepoints = torch.as_tensor(np.stack([raceline_np[k] for k in ["x", "y",]], axis=1), dtype=torch.float64)#, device=torch.device("cuda:0"))
        gpu = self.get_parameter(RacelinePropagator.GPU_PARAMETER_NAME).get_parameter_value().integer_value
        if gpu>=0: racelinepoints = racelinepoints.cuda(gpu)
        timescale = self.get_parameter(RacelinePropagator.TIMESCALE_PARAMETER_NAME).get_parameter_value().double_value
        racelinespeeds = timescale*torch.as_tensor(raceline_np["speed"]).type_as(racelinepoints)
        self.get_logger().info("Building raceline helper")
        raceline_helper = mu.RacelineHelper.from_closed_path(racelinepoints, racelinespeeds, 1.0).to(tensor=racelinepoints)#.float()
        self.get_logger().info("Built raceline helper")

        prediction_horizon = self.get_parameter(RacelinePropagator.PREDICTION_HORIZON_PARAMETER_NAME).get_parameter_value().double_value
        self.tdelta = torch.linspace(0.0, prediction_horizon, steps=30).type_as(raceline_helper.__arclengths_in__)


        self.get_logger().info("Compiling raceline helper")
        with torch.no_grad():
            self.raceline_helper = torch.compile(raceline_helper, mode="max-autotune-no-cudagraphs", fullgraph=True)
            self.raceline_helper.__t_of_r__.compile(mode="max-autotune-no-cudagraphs", fullgraph=True, dynamic=True)
            dummyt = self.tdelta + self.raceline_helper.__r_of_t__.xstart_vec[torch.randint(0, high=self.raceline_helper.__r_of_t__.xstart_vec.shape[0], size=[1,]).item()].item()
            dummyr, _, _, _ = self.raceline_helper(t=dummyt)
            self.raceline_helper(r=dummyr)

            idx_rand = int(np.random.randint(0, high=racelinepoints.shape[0], size=[1,])[0])
            prand = racelinepoints[idx_rand].clone()
            prand += 2.5*torch.randn_like(prand)


            closest_r, _, _, _ = self.raceline_helper.closest_point_approximate(prand[None], newton_iterations=4)

            closest_t = self.raceline_helper.t_of_r(closest_r)

            print("closest_t: %s" % (str(closest_t),))




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
                # irand = np.random.randint(0, high=self.raceline_helper.__r_of_t__.xstart_vec.shape[0], size=[1,]).item()
                # trand = self.raceline_helper.__r_of_t__.xstart_vec[irand]
                yay = self.raceline_helper(t=(self.tdelta + tequavalent[0].item()))
                tock = time.time()
                comptimes[i] = (tock - tick)
            self.get_logger().info("comptimes: %s" % (str(1000.0*comptimes),))
        stateparam = rclpy.Parameter(RacelinePropagator.STATE_PARAMETER_NAME, rclpy.Parameter.Type.STRING, "PREDICTING")
        self.set_parameters([stateparam,])


        # print(pointssamp[0])
        



        

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = RacelinePropagator()
    getline_client : rclpy.client.Client = node.create_client(deepracing_srvs.GetLine, "get_line")
    node.get_logger().info("Waiting for get_line service...")
    getline_client.wait_for_service()
    node.get_logger().info("Got get_line service...")
    getlinereq : deepracing_srvs.GetLine.Request = deepracing_srvs.GetLine.Request()
    getlinereq.key.data="raceline"
    success = False
    while not success:
        future = getline_client.call_async(getlinereq)
        rclpy.spin_until_future_complete(node, future, timeout_sec = 2.0)
        getlineresponse : deepracing_srvs.GetLine.Response = future.result()
        if getlineresponse is None:
            node.get_logger().error("Get line service call returned None. Retrying...")
        elif getlineresponse.return_code==deepracing_srvs.GetLine.Response.SUCCESS:
            success = True
            node.get_logger().info("Successfully got the raceline")
        else:
            node.get_logger().error("Unable to get the raceline. Error code: %d." % (getlineresponse.return_code,))
            # exit(-1)
    racelinenp = sensor_msgs_py.point_cloud2.read_points(getlineresponse.line)
    initialize_thread = threading.Thread(
        target=node.initialize,
        args=(racelinenp,)
    )
    initialize_thread.start()
    rclpy.spin(node)#, rclpy.executors.MultiThreadedExecutor(num_threads=3))

if __name__ == '__main__':
    main()