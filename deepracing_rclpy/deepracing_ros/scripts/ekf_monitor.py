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

import ament_index_python
import rclpy
import rclpy.executors
import rclpy.node 
import rclpy.publisher  
import rclpy.subscription  
import rclpy.logging  
import rclpy.duration
import rclpy.time
import nav_msgs.msg
import geometry_msgs.msg
import robot_localization.msg
import deepracing_msgs.msg  
from scipy.spatial.transform import Rotation
import deepracing_ros.utils
import deepracing_ros.convert
import deepracing
import rclpy.executors 
import tf2_ros

class EKFMonitor(rclpy.node.Node):
    def __init__(self, name="ekf_monitor"):
        super(EKFMonitor, self).__init__(name)
        self.motion_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketMotionData, "motion_data", self.motionDataCB, 10)
        self.odom_sub : rclpy.subscription.Subscription = self.create_subscription(nav_msgs.msg.Odometry, "odom", self.odomCB, 10)
        self.odom_filtered_sub : rclpy.subscription.Subscription = self.create_subscription(nav_msgs.msg.Odometry, "odom/filtered", self.odomFilteredCB, 10)
        self.set_state_pub : rclpy.publisher.Publisher = self.create_publisher(robot_localization.msg.State, "set_state", 1)
        self.prev_motion_data : deepracing_msgs.msg.TimestampedPacketMotionData = None
        covariances_file : str = deepracing.searchForFile("covariances.json", [os.curdir, os.path.join(ament_index_python.get_package_share_directory("deepracing_launch"), "data")] )
        with open(covariances_file, "r") as f:
            covariance_dict : dict = json.load(f)
        self.pose_cov : np.ndarray = np.eye(6, dtype=np.float64)
        self.pose_cov[0:3,0:3] = np.asarray(covariance_dict["position_cov"], dtype=self.pose_cov.dtype).reshape(self.pose_cov[0:3,0:3].shape)
        self.pose_cov[3:,3:] = np.asarray(covariance_dict["euler_angles_cov"], dtype=self.pose_cov.dtype).reshape(self.pose_cov[0:3,0:3].shape)
        self.twist_cov : np.ndarray = np.eye(6, dtype=self.pose_cov.dtype)
        self.twist_cov[0:3,0:3] = np.asarray(covariance_dict["linear_vel_cov"], dtype=self.twist_cov.dtype).reshape(self.twist_cov[0:3,0:3].shape)
        self.twist_cov[3:,3:] = np.asarray(covariance_dict["angular_vel_cov"], dtype=self.twist_cov.dtype).reshape(self.twist_cov[0:3,0:3].shape)
        self.accel_cov : np.ndarray = np.eye(6, dtype=self.pose_cov.dtype)
        self.accel_cov[0:3,0:3] = np.asarray(covariance_dict["linear_accel_cov"], dtype=self.accel_cov.dtype).reshape(self.accel_cov[0:3,0:3].shape)
        self.accel_cov[3:,3:] = -np.ones_like(self.accel_cov[3:,3:])
        self.tf2_buffer : tf2_ros.Buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.tf2_listener : tf2_ros.TransformListener = tf2_ros.TransformListener(self.tf2_buffer, self, spin_thread=False)

    def timerCB(self):
        if self.prev_motion_data is None:
            return
        wallclocknow : rclpy.time.Time = self.get_clock().now()
        dt : rclpy.duration.Duration = wallclocknow - rclpy.time.Time.from_msg(self.prev_motion_data.header.stamp)
        dtfloat : float = float(dt.nanoseconds*1.0E-9)
        if dtfloat>1.0:
            self.get_logger().info("Pause detected, resetting EKF state")
            self.publishState(self.prev_motion_data)
    def odomCB(self, odom : nav_msgs.msg.Odometry):
        pass
    def odomFilteredCB(self, odom_filtered : nav_msgs.msg.Odometry):
        pass
    def publishState(self, motion_data : deepracing_msgs.msg.TimestampedPacketMotionData):
        transform_stamped_msg : geometry_msgs.msg.TransformStamped = self.tf2_buffer.lookup_transform("map", "track", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5))
        translation_msg : geometry_msgs.msg.Vector3 = transform_stamped_msg.transform.translation
        rotation_msg : geometry_msgs.msg.Quaternion = transform_stamped_msg.transform.rotation
        mapToTrack : np.ndarray = np.eye(4, dtype=self.pose_cov.dtype)
        mapToTrack[0:3,0:3] = Rotation.from_quat([rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w]).as_matrix().astype(mapToTrack.dtype)
        mapToTrack[0:3,3] = np.asarray([translation_msg.x, translation_msg.y, translation_msg.z], dtype=mapToTrack.dtype)
        current_position : np.ndarray = deepracing_ros.convert.extractPosition(motion_data.udp_packet)
        current_rotation : Rotation = deepracing_ros.convert.extractOrientation(motion_data.udp_packet)
        trackToCar : np.ndarray = np.eye(4, dtype=mapToTrack.dtype)
        trackToCar[0:3,0:3] = current_rotation.as_matrix().astype(trackToCar.dtype)
        trackToCar[0:3,3] = current_position.astype(trackToCar.dtype)
        poseMap : np.ndarray = np.matmul(mapToTrack, trackToCar)
        current_linear_vel : np.ndarray = np.matmul(mapToTrack[0:3,0:3], deepracing_ros.convert.extractVelocity(motion_data.udp_packet))
        current_angular_vel : np.ndarray = np.matmul(mapToTrack[0:3,0:3], deepracing_ros.convert.extractAngularVelocity(motion_data.udp_packet))
        current_accel : np.ndarray = np.matmul(poseMap[0:3,0:3], deepracing_ros.convert.extractAcceleration(motion_data.udp_packet))
        state_to_pub : robot_localization.msg.State = robot_localization.msg.State()
        state_to_pub.update_vector = [True for asdf in range(15)]
        state_to_pub.pose = geometry_msgs.msg.PoseWithCovarianceStamped(header=motion_data.header)
        state_to_pub.twist = geometry_msgs.msg.TwistWithCovarianceStamped(header=motion_data.header)
        state_to_pub.accel = geometry_msgs.msg.AccelWithCovarianceStamped(header=motion_data.header)
        state_to_pub.pose.header.frame_id=state_to_pub.twist.header.frame_id=state_to_pub.accel.header.frame_id="map"
        state_to_pub.pose.pose.pose.position = geometry_msgs.msg.Point(x = poseMap[0,3], y = poseMap[1,3], z = poseMap[2,3])
        poseMapQuat : np.ndarray = Rotation.from_matrix(poseMap[0:3,0:3]).as_quat()
        state_to_pub.pose.pose.pose.orientation = geometry_msgs.msg.Quaternion(x=poseMapQuat[0], y=poseMapQuat[1], z=poseMapQuat[2], w=poseMapQuat[3])
        state_to_pub.pose.pose.covariance = self.pose_cov.flatten()
        state_to_pub.twist.twist.twist.linear= geometry_msgs.msg.Vector3(x = current_linear_vel[0], y = current_linear_vel[1], z = current_linear_vel[2])
        state_to_pub.twist.twist.twist.angular= geometry_msgs.msg.Vector3(x = current_angular_vel[0], y = current_angular_vel[1], z = current_angular_vel[2])
        state_to_pub.twist.twist.covariance = self.twist_cov.flatten()
        state_to_pub.accel.accel.accel.linear= geometry_msgs.msg.Vector3(x = current_accel[0], y = current_accel[1], z = current_accel[2])
        state_to_pub.accel.accel.covariance = self.accel_cov.flatten()
        self.set_state_pub.publish(state_to_pub)
    def motionDataCB(self, motion_data : deepracing_msgs.msg.TimestampedPacketMotionData):
        if self.prev_motion_data is None:
            self.prev_motion_data = copy.deepcopy(motion_data)
            self.get_logger().info("First data packet received, initializing EKF state")
            self.publishState(self.prev_motion_data)
            return        
        previous_position : np.ndarray = deepracing_ros.convert.extractPosition(self.prev_motion_data.udp_packet)
        previous_linear_velocity : np.ndarray = deepracing_ros.convert.extractVelocity(self.prev_motion_data.udp_packet)
        tprevious : float = self.prev_motion_data.udp_packet.header.session_time
        current_position : np.ndarray = deepracing_ros.convert.extractPosition(motion_data.udp_packet)
        tcurrent : float = motion_data.udp_packet.header.session_time
        dt : float = tcurrent - tprevious
        predicted_position : np.ndarray = previous_position + dt*previous_linear_velocity
        delta_pos : np.ndarray =  predicted_position - current_position
        if np.linalg.norm(delta_pos, ord=2, axis=0)>10.0:
            self.get_logger().info("Unexpected jump in car position, probably a lap reset.  Resetting EKF state")
            self.publishState(motion_data)
        self.prev_motion_data = copy.deepcopy(motion_data)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = EKFMonitor()
    node.create_timer(0.5, node.timerCB)
    rclpy.spin(node, rclpy.executors.MultiThreadedExecutor())

if __name__ == '__main__':
    main()