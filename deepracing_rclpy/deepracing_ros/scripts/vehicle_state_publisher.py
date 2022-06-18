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

from email.header import Header
from turtle import shape
import ament_index_python
import os
import rclpy
import time
import rclpy
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from std_msgs.msg import String
from deepracing_msgs.msg import TimestampedPacketMotionData, PacketMotionData, CarMotionData, TimestampedPacketLapData, PacketLapData, LapData, TimestampedPacketSessionData, PacketSessionData
from deepracing_msgs.msg import CarControl, DriverStates
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion, TwistStamped, Twist, TransformStamped, Transform, Vector3Stamped, Vector3
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import rclpy.executors
import rclpy.duration
import rclpy.time
import tf2_ros
from deepracing_ros.utils import AsyncSpinner
from scipy.spatial.transform import Rotation as Rot
from threading import Semaphore
import deepracing
import deepracing_ros, deepracing_ros.convert as C
import deepracing.protobuf_utils
from typing import List
import shapely, shapely.geometry
import json

class DriverStatePublisher(Node):
    def __init__(self,):
        super(DriverStatePublisher,self).__init__('driver_state_publisher')
        self.lap_data_semaphore : Semaphore = Semaphore()
        self.valid_indices : np.ndarray = None
        self.lap_data_sub : Subscription = self.create_subscription(TimestampedPacketLapData, "lap_data", self.lapDataCB, 1)
        self.motion_data_sub : Subscription = self.create_subscription(TimestampedPacketMotionData, "motion_data", self.motionPacketCB, 1)
        self.session_data_sub : Subscription = self.create_subscription(TimestampedPacketSessionData, "session_data", self.sessionDataCB, 1)

        self.pose_array_pub : Publisher = self.create_publisher(PoseArray, "driver_poses", 1)
        self.driver_state_pub : Publisher = self.create_publisher(DriverStates, "driver_states", 1)

        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)

        self.transform : np.ndarray = None
        self.centerline : np.ndarray = None
        self.ring_length : float = 0.0
        self.centerline_ring : shapely.geometry.LinearRing = None


    def sessionDataCB(self, timestamped_session_data : TimestampedPacketSessionData):
        if self.centerline_ring is None:
            session_data : PacketSessionData = timestamped_session_data.udp_packet
            trackname : str = deepracing.trackNames[session_data.track_id]
            env_track_dirs = os.getenv("F1_TRACK_DIRS", None)
            if env_track_dirs is not None:
                search_dirs : List[str] = env_track_dirs.split(os.pathsep)
            else:
                search_dirs : List[str] = []
            try:
                search_dirs.append(os.path.join(ament_index_python.get_package_share_directory("f1_datalogger"), "f1_tracks"))
            except Exception as e:
                pass
            centerline_file : str = deepracing.searchForFile("%s_centerline.json" %(trackname,), search_dirs)
            with open(centerline_file, "r") as f:
                d : dict = json.load(f)
            centerline : np.ndarray = np.row_stack([d["xin"], d["yin"], d["zin"], np.ones_like(d["zin"])]).astype(np.float64)
            transform_stamped_msg : TransformStamped = self.tf2_buffer.lookup_transform("map", "track", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3))
            transform_msg : Transform = transform_stamped_msg.transform
            self.transform = np.eye(4, dtype=centerline.dtype)
            self.transform[0:3,0:3] = Rot.from_quat(np.asarray([transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z, transform_msg.rotation.w], dtype=centerline.dtype)).as_matrix()
            self.transform[0:3,3] = np.asarray([transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z], dtype=centerline.dtype)
            self.centerline = np.matmul(self.transform, centerline)[0:3].T
            self.centerline_ring = shapely.geometry.LinearRing(self.centerline[:,[0,1]].tolist())
            self.ring_length = self.centerline_ring.length
            self.destroy_subscription(self.session_data_sub)
    def lapDataCB(self, timestamped_lap_data : TimestampedPacketLapData):
        ego_idx = timestamped_lap_data.udp_packet.header.player_car_index
        if not self.lap_data_semaphore.acquire(timeout = 2.0):
            self.get_logger().error("Unable to acquire lap_data_semaphore in lapDataCB")
            return
        self.valid_indices : np.ndarray = np.asarray( [i for i in range(len(timestamped_lap_data.udp_packet.lap_data)) if ( (timestamped_lap_data.udp_packet.lap_data[i].result_status not in {0,1}) and (not i==ego_idx) ) ] , dtype=np.int32)
        self.lap_data_semaphore.release()
    def motionPacketCB(self, timestamped_motion_data : TimestampedPacketMotionData):
        if self.centerline_ring is None:
            self.get_logger().error("Have not yet received any session data to infer track data")
            return
        ego_idx = timestamped_motion_data.udp_packet.header.player_car_index
        if self.valid_indices is None:
            self.get_logger().error("Have not yet received any lap data to infer vehicle indices")
            return
        self.get_logger().debug("Processing motion packet")
        udp_packet : PacketMotionData = timestamped_motion_data.udp_packet
        motion_data_array : List[CarMotionData] = udp_packet.car_motion_data
        if not self.lap_data_semaphore.acquire(timeout = 2.0):
            self.get_logger().error("Unable to acquire lap_data_semaphore in lapDataCB")
            return
        valid_indices = self.valid_indices.copy()
        self.lap_data_semaphore.release()
        driver_states : DriverStates = DriverStates(ego_vehicle_index = ego_idx)
        driver_states.header.stamp = timestamped_motion_data.header.stamp
        driver_states.header.frame_id="map"
        pose_array : PoseArray = PoseArray()
        pose_array.header.stamp = timestamped_motion_data.header.stamp
        pose_array.header.frame_id="map"
        posetrack : np.ndarray = np.eye(4, dtype=self.centerline.dtype)
        linearveltrack : np.ndarray = np.zeros_like(posetrack[0:3,3])
        for i in range(valid_indices.shape[0]):
            car_index = valid_indices[i]
            rottrack = C.extractOrientation(udp_packet, car_index=car_index)
            posetrack[0:3,0:3] = rottrack.as_matrix()
            positiontrackmsg : Point = motion_data_array[car_index].world_position.point
            posetrack[0:3,3] = np.asarray([positiontrackmsg.x, positiontrackmsg.y, positiontrackmsg.z], dtype=posetrack.dtype)
            posemap : np.ndarray = np.matmul(self.transform, posetrack)
            positionmapmsg : Point = Point(x=posemap[0,3], y=posemap[1,3], z=posemap[2,3])
            quatmap : np.ndarray = Rot.from_matrix(posemap[0:3,0:3]).as_quat()
            pose = Pose(position = positionmapmsg, orientation=Quaternion(x=quatmap[0], y=quatmap[1], z=quatmap[2], w=quatmap[3]))        
            pose_array.poses.append(pose)
            driver_states.other_agent_poses.append(pose)
            linearveltrack : np.ndarray = np.asarray([motion_data_array[car_index].world_velocity.vector.x, motion_data_array[car_index].world_velocity.vector.y, motion_data_array[car_index].world_velocity.vector.z], dtype=linearveltrack.dtype)
            linearvelmap : np.ndarray = np.matmul(self.transform[0:3,0:3], linearveltrack)
            driver_states.other_agent_velocities.append(Vector3(x=linearvelmap[0], y=linearvelmap[1], z=linearvelmap[2]))
            driver_states.vehicle_indices.append(car_index)
            ptshapely : shapely.geometry.Point = shapely.geometry.Point(positionmapmsg.x, positionmapmsg.y)
            ring_distance : float = self.centerline_ring.project(ptshapely)
            if ring_distance>self.ring_length/2.0:
                ring_distance-=self.ring_length
            driver_states.other_agent_track_progress.append(ring_distance)
        #now grab data for ego vehicle
        ego_rotation = C.extractOrientation(udp_packet)
        posetrack[0:3,0:3] = ego_rotation.as_matrix()
        positiontrackmsg : Point = motion_data_array[ego_idx].world_position.point
        posetrack[0:3,3] = np.asarray([positiontrackmsg.x, positiontrackmsg.y, positiontrackmsg.z], dtype=posetrack.dtype)
        posemap : np.ndarray = np.matmul(self.transform, posetrack)
        positionmapmsg : Point = Point(x=posemap[0,3], y=posemap[1,3], z=posemap[2,3])
        quatmap : np.ndarray = Rot.from_matrix(posemap[0:3,0:3]).as_quat()
        driver_states.ego_pose = Pose(position = positionmapmsg, orientation=Quaternion(x=quatmap[0], y=quatmap[1], z=quatmap[2], w=quatmap[3]))
        
        linearveltrack : np.ndarray = np.asarray([motion_data_array[ego_idx].world_velocity.vector.x, motion_data_array[ego_idx].world_velocity.vector.y, motion_data_array[ego_idx].world_velocity.vector.z], dtype=linearveltrack.dtype)
        angularveltrack : np.ndarray = np.asarray([udp_packet.angular_velocity.x, udp_packet.angular_velocity.y, udp_packet.angular_velocity.z], dtype=linearveltrack.dtype)
        linearvelmap : np.ndarray = np.matmul(self.transform[0:3,0:3], linearveltrack)
        angularvelmap : np.ndarray = np.matmul(self.transform[0:3,0:3], angularveltrack)
        driver_states.ego_velocity.linear = Vector3(x=linearvelmap[0], y=linearvelmap[1], z=linearvelmap[2]) 
        driver_states.ego_velocity.angular = Vector3(x=angularvelmap[0], y=angularvelmap[1], z=angularvelmap[2]) 
        ptshapely : shapely.geometry.Point = shapely.geometry.Point(positionmapmsg.x, positionmapmsg.y)
        ring_distance : float = self.centerline_ring.project(ptshapely)
        if ring_distance>self.ring_length/2.0:
            ring_distance-=self.ring_length
        driver_states.ego_track_progress=ring_distance
        self.pose_array_pub.publish(pose_array)
        self.driver_state_pub.publish(driver_states)



    
def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = DriverStatePublisher()
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