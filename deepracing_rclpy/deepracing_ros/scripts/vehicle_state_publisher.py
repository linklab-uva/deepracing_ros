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

import rclpy
import time
import rclpy
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.node import Node
from std_msgs.msg import String
from deepracing_msgs.msg import TimestampedPacketMotionData, PacketMotionData, CarMotionData, TimestampedPacketLapData, PacketLapData, LapData
from deepracing_msgs.msg import CarControl, DriverStates
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Quaternion, Twist
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import rclpy.executors
from deepracing_ros.utils import AsyncSpinner
from scipy.spatial.transform import Rotation as Rot
from threading import Semaphore
import deepracing_ros, deepracing_ros.convert as C
import deepracing.protobuf_utils
from typing import List

class DriverStatePublisher(Node):
    def __init__(self,):
        super(DriverStatePublisher,self).__init__('driver_state_publisher')
        self.lap_data_semaphore : Semaphore = Semaphore()
        self.valid_indices : np.ndarray = None
        self.lap_data_sub : Subscription = self.create_subscription(TimestampedPacketLapData, "lap_data", self.lapDataCB, 1)
        self.motion_data_sub : Subscription = self.create_subscription(TimestampedPacketMotionData, "motion_data", self.motionPacketCB, 1)

        self.pose_array_pub : Publisher = self.create_publisher(PoseArray, "driver_poses", 1)
        self.driver_state_pub : Publisher = self.create_publisher(DriverStates, "driver_states", 1)


    def lapDataCB(self, timestamped_lap_data : TimestampedPacketLapData):
        ego_idx = timestamped_lap_data.udp_packet.header.player_car_index
        if not self.lap_data_semaphore.acquire(timeout = 2.0):
            self.get_logger().error("Unable to acquire lap_data_semaphore in lapDataCB")
            return
        self.valid_indices : np.ndarray = np.asarray( [i for i in range(len(timestamped_lap_data.udp_packet.lap_data)) if ( (timestamped_lap_data.udp_packet.lap_data[i].result_status not in {0,1}) and (not i==ego_idx) ) ] , dtype=np.int32)
        self.lap_data_semaphore.release()
    def motionPacketCB(self, timestamped_motion_data : TimestampedPacketMotionData):
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
        driver_states : DriverStates = DriverStates(header = timestamped_motion_data.header, ego_vehicle_index = ego_idx)
        pose_array : PoseArray = PoseArray(header = timestamped_motion_data.header)
        for i in range(valid_indices.shape[0]):
            car_index = valid_indices[i]
            qnp = C.extractOrientation(udp_packet, car_index=car_index).as_quat() 
            pose = Pose(position = motion_data_array[car_index].world_position.point, orientation=Quaternion(x=qnp[0], y=qnp[1], z=qnp[2], w=qnp[3]))        
            pose_array.poses.append(pose)
            driver_states.other_agent_poses.append(pose)
            driver_states.other_agent_velocities.append(motion_data_array[car_index].world_velocity.vector)
            driver_states.vehicle_indices.append(car_index)
        #now grab data for ego vehicle
        ego_rotation = C.extractOrientation(udp_packet)
        ego_quaternion = ego_rotation.as_quat()
        driver_states.ego_pose = Pose(position = motion_data_array[ego_idx].world_position.point, orientation=Quaternion(x=ego_quaternion[0], y=ego_quaternion[1], z=ego_quaternion[2], w=ego_quaternion[3]))
        driver_states.ego_velocity = Twist(linear = motion_data_array[ego_idx].world_velocity.vector, angular = udp_packet.angular_velocity)
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