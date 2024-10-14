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

from typing import Union
import rclpy
import rclpy.timer
import rclpy.client
import rclpy.subscription
import rclpy.executors
import rclpy.timer
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String, Float64
from nav_msgs.msg import Path
import deepracing_msgs.srv
from deepracing_msgs.msg import TimestampedPacketMotionData, CarMotionData, CarControl, BezierCurve, TimestampedPacketSessionData
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from deepracing_ros.controls.path_server_oracle_raceline import OraclePathServer
from deepracing_ros.utils import AsyncSpinner
from rclpy.executors import MultiThreadedExecutor
import ament_index_python
import os
import deepracing

class RacelineSetter(Node):
    def __init__(self, name="initial_raceline_setter"):
        super(RacelineSetter, self).__init__(name)
        self.session_sub : rclpy.subscription.Subscription = self.create_subscription(TimestampedPacketSessionData, '/session_data', self.sessionCallback, 1)
        self.current_session_data : TimestampedPacketSessionData = None

    def sessionCallback(self, session_msg : TimestampedPacketSessionData):
        self.get_logger().debug("Got a new session packet: " + str(session_msg))
        self.current_session_data = session_msg

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = RacelineSetter()
    default_trackfile_param : rclpy.Parameter = node.declare_parameter("default_trackfile", value="")
    default_trackfile : str = default_trackfile_param.get_parameter_value().string_value
    if default_trackfile=="":
        node.get_logger().info("default_trackfile parameter not set, exiting")
        exit(0)
    f1_track_env = os.getenv("F1_TRACK_DIRS")
    if f1_track_env is not None:
        search_dirs = str.split(f1_track_env, os.pathsep)
    else:
        search_dirs = []
    exec : rclpy.executors.SingleThreadedExecutor = rclpy.executors.SingleThreadedExecutor()
    asynspinner : AsyncSpinner = AsyncSpinner(exec)
    asynspinner.add_node(node)
    asynspinner.spin()
    rate : rclpy.timer.Rate = node.create_rate(1.0, clock=node.get_clock())
    while node.current_session_data is None:
        node.get_logger().info("Waiting for session data")
        rate.sleep()
    
    trackname = deepracing.trackNames[node.current_session_data.udp_packet.track_id]
    node.get_logger().info("Got track name: %s" % (trackname,))

    mapdir : str = os.path.join(ament_index_python.get_package_share_directory("deepracing_launch"), "maps")
    search_dirs.append(mapdir)

    trackmap : deepracing.TrackMap  = deepracing.searchForTrackmap(trackname, search_dirs, align=True, transform_to_map=True)
    if trackmap is None:
        node.get_logger().error("Could not find trackmap for %s in any of %s" % (trackname, str(search_dirs)))
        exit(-1)

    linedict : dict = trackmap.linemap[default_trackfile]

    node.get_logger().info("Setting raceline to %s" % (linedict["filepath"],))
    serviceclient : rclpy.client.Client = node.create_client(deepracing_msgs.srv.SetRaceline, "set_raceline")
    serviceclient.wait_for_service()
    req : deepracing_msgs.srv.SetRaceline.Request = deepracing_msgs.srv.SetRaceline.Request()
    req.filename=linedict["filepath"]
    req.frame_id="track"
    success = False
    while not success:
        future = serviceclient.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        # while not future.done():
        #     rate.sleep()
        response : deepracing_msgs.srv.SetRaceline.Response = future.result()
        if response.error_code==deepracing_msgs.srv.SetRaceline.Response.SUCCESS:
            success = True
            node.get_logger().info("Successfully set the raceline")
        else:
            node.get_logger().error("Unable to set the raceline. Error code: %d. Error message: %s" % (response.error_code, response.message))


if __name__ == '__main__':
    main()