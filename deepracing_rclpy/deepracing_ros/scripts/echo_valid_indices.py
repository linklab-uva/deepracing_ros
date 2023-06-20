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
import rclpy.logging
import rclpy.subscription  
import rclpy.node
import deepracing_msgs.msg  
import threading
import copy
from typing import Sequence
class EchoValidIndices(rclpy.node.Node):
    def __init__(self, name="echo_valid_indices"):
        super(EchoValidIndices, self).__init__(name)
        self.lap_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketLapData, "/lap_data", self.lapDataCB, 10)
        self.current_lap_data : deepracing_msgs.msg.TimestampedPacketLapData = None
        self.data_mutex : threading.Semaphore = threading.Semaphore()
    def timerCB(self):
        if self.current_lap_data is None:
            return
        if not self.data_mutex.acquire(timeout=2.0):
            self.get_logger().error("Unable to acquire semaphore")
            return
        current_lap_data : deepracing_msgs.msg.TimestampedPacketLapData = copy.deepcopy(self.current_lap_data)
        self.data_mutex.release()
        lap_data_array : Sequence[deepracing_msgs.msg.LapData] = current_lap_data.udp_packet.lap_data
        ego_idx : int = current_lap_data.udp_packet.header.player_car_index
        secondary_idx : int = current_lap_data.udp_packet.header.secondary_player_car_index
        valid_indices = []
        if ego_idx>=0 and ego_idx<len(lap_data_array) and lap_data_array[ego_idx].result_status not in {0,1}:
            valid_indices.append("%d (Player Car)" % (ego_idx,))
        if secondary_idx>=0 and secondary_idx<len(lap_data_array) and lap_data_array[secondary_idx].result_status not in {0,1}:
            valid_indices.append("%d (Secondary Player Car)" % (secondary_idx,))
        for idx in range(len(lap_data_array)):
            if (idx not in {ego_idx, secondary_idx}) and (lap_data_array[idx].result_status not in {0,1}):
                valid_indices.append("%d" % (idx,))
        self.get_logger().info("These vehicle indices are producing non-garbage data: %s" % (str(valid_indices),))
        
    def lapDataCB(self, lap_data : deepracing_msgs.msg.TimestampedPacketLapData):
        if not self.data_mutex.acquire(timeout=2.0):
            self.get_logger().error("Unable to acquire semaphore")
            return
        self.current_lap_data = lap_data
        self.data_mutex.release()

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = EchoValidIndices()
    node.create_timer(2.0, node.timerCB)
    rclpy.spin(node, rclpy.executors.MultiThreadedExecutor())

if __name__ == '__main__':
    main()