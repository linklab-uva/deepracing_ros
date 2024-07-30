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
import rclpy.timer
import rclpy.client
import rclpy.executors
import rclpy.parameter
import rclpy.exceptions
from rclpy.node import Node
import lifecycle_msgs.srv, lifecycle_msgs.msg

class InitializeLifecycleNode(Node):
    def __init__(self, name="initialize_lifecycle_node"):
        super(InitializeLifecycleNode, self).__init__(name)
        node_to_initialize_param = self.declare_parameter("node_to_initialize")
        self.node_to_initialize : str = node_to_initialize_param.get_parameter_value().string_value


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = InitializeLifecycleNode()
    if node.node_to_initialize is None:
        raise ValueError("node_to_initialize param is not set")
    service_name = "/%s/change_state" % (node.node_to_initialize,)
    serviceclient : rclpy.client.Client = node.create_client(lifecycle_msgs.srv.ChangeState, service_name)
    serviceclient.wait_for_service()
    req : lifecycle_msgs.srv.ChangeState.Request = lifecycle_msgs.srv.ChangeState.Request()

    req.transition.label="configure"
    req.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    future = serviceclient.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response : lifecycle_msgs.srv.ChangeState.Response = future.result()
    if not response.success:
        node.get_logger().error("Unable to configure %s" % (node.node_to_initialize,))
        rclpy.shutdown()
        exit(-1)
    req.transition.label="activate"
    req.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
    future = serviceclient.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response : lifecycle_msgs.srv.ChangeState.Response = future.result()
    if not response.success:
        node.get_logger().error("Unable to activate %s" % (node.node_to_initialize,))
        rclpy.shutdown()
        exit(-1)
    exit(0)

if __name__ == '__main__':
    main()