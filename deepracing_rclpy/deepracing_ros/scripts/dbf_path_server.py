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

import rclpy.exceptions
import rclpy, rclpy.logging, rclpy.client, rclpy.node
import rclpy.timer
import numpy as np
import sensor_msgs_py.point_cloud2
import deepracing_msgs.srv as deepracing_srvs
from deepracing_ros.controls.path_server_dbfovertaking import DBFOvertakingPathServer
import threading
def call_line_service(getline_client : rclpy.client.Client, node : rclpy.node.Node, key : str):
    getlinereq : deepracing_srvs.GetLine.Request = deepracing_srvs.GetLine.Request()
    success = False
    getlinereq.key.data=key
    while not success:
        future = getline_client.call_async(getlinereq)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        getlineresponse : deepracing_srvs.GetLine.Response = future.result()
        if getlineresponse is None:
            node.get_logger().error("Get line service call returned None. Retrying...")
        elif getlineresponse.return_code==deepracing_srvs.GetLine.Response.SUCCESS:
            success = True
            node.get_logger().info("Successfully got line with key %s" % (getlinereq.key.data,))
        else:
            node.get_logger().error("Unable to get line with key %s. Error code: %d." % (getlinereq.key.data,getlineresponse.return_code))
            #exit(-1)
    line_np : np.ndarray = sensor_msgs_py.point_cloud2.read_points(getlineresponse.line)
    line_frame : str = getlineresponse.line.header.frame_id
    return line_np, line_frame
def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = DBFOvertakingPathServer()
    frequency_param : rclpy.Parameter = node.declare_parameter("rate", value=100.0)
    try:
        print(node.params.compile_backend)
    except rclpy.exceptions.ParameterUninitializedException as e:
        print("No compile backend set")

    


    getline_client : rclpy.client.Client = node.create_client(deepracing_srvs.GetLine, "get_line")
    getline_client.wait_for_service()
    
    raceline_np, raceline_frame = call_line_service(getline_client, node, "raceline")
    innerbound_np, innerbound_frame = call_line_service(getline_client, node, "inner_bound")
    outerbound_np, outerbound_frame = call_line_service(getline_client, node, "outer_bound")
    width_map_np, width_map_frame = call_line_service(getline_client, node, "width_map")


    node.get_logger().info("raceline_frame: %s" % (raceline_frame,))
    node.get_logger().info("innerbound_frame: %s" % (innerbound_frame,))
    node.get_logger().info("outerbound_frame: %s" % (outerbound_frame,))
    node.get_logger().info("width_map_frame: %s" % (width_map_frame,))

    initialize_thread = threading.Thread(
        target=node.initialize,
        args=(raceline_np, width_map_np, innerbound_np, outerbound_np),
        # daemon=True
    )
    initialize_thread.start()
    # node.initialize(raceline_np, width_map_np, innerbound_np, outerbound_np)
    # initialize_thread.
    timer : rclpy.timer.Timer = node.create_timer(1.0/frequency_param.get_parameter_value().double_value, node.getTrajectory)

    rclpy.spin(node)
    


if __name__ == '__main__':
    main()