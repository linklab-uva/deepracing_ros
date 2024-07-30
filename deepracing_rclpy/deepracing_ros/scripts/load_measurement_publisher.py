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
import rcl_interfaces.msg
import composition_interfaces.srv

class LoadMeasurementPublisher(Node):
    def __init__(self, name="initialize_lifecycle_node"):
        super(LoadMeasurementPublisher, self).__init__(name)
        node_container_name_desc : rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor()
        node_container_name_desc.name = "node_container_name"
        node_container_name_desc.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING
        node_container_name_param = self.declare_parameter(node_container_name_desc.name, descriptor=node_container_name_desc, value="")
        self.node_container_name : str | None = node_container_name_param.get_parameter_value().string_value

        car_index_desc : rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor()
        car_index_desc.name = "car_index"
        car_index_desc.type = rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER
        car_index_param = self.declare_parameter(car_index_desc.name, descriptor=car_index_desc, value=-1)
        self.car_index : int | None = car_index_param.get_parameter_value().integer_value


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = LoadMeasurementPublisher()
    if (node.node_container_name is None) or (node.node_container_name == ""):
        raise ValueError("node_container_name param is not set")
    if (node.car_index is None) or (node.car_index < 0):
        raise ValueError("car_index param is not set")
    
    service_name = "%s/_container/load_node" % (node.node_container_name,)
    serviceclient : rclpy.client.Client = node.create_client(composition_interfaces.srv.LoadNode, service_name)
    serviceclient.wait_for_service()
    req : composition_interfaces.srv.LoadNode.Request = composition_interfaces.srv.LoadNode.Request()
    req.log_level = 1
    req.node_name = "measurement_pub"
    req.package_name="deepracing_rclcpp"
    req.plugin_name="deepracing::composable_nodes::MeasurementPublisher"
    car_name="car%d" % (node.car_index,)
    req.node_namespace="/%s" % (car_name,)
    param_car_name : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
    param_car_name.name="carname"
    param_car_name.value.string_value = car_name
    param_car_name.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING
    req.parameters.append(param_car_name)
    param_car_index : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
    param_car_index.name="index"
    param_car_index.value.integer_value = node.car_index
    param_car_index.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER
    req.parameters.append(param_car_index)
    
    future = serviceclient.call_async(req)
    rclpy.spin_until_future_complete(node, future)

if __name__ == '__main__':
    main()