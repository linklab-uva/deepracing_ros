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
import rcl_interfaces.msg
import yaml
import composition_interfaces.srv

class InitializerUDPRecevier(Node):
    def __init__(self, name="initialize_udp_receiver"):
        super(InitializerUDPRecevier, self).__init__(name)

        drivers_file_desc : rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor()
        drivers_file_desc.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING
        drivers_file_desc.name = "drivers_file"
        drivers_file_param : rclpy.parameter.Parameter = self.declare_parameter(drivers_file_desc.name, descriptor=drivers_file_desc)
        self.drivers_file : str = drivers_file_param.get_parameter_value().string_value
        with open(self.drivers_file, "r") as f:
            self.drivers : dict = yaml.load(f, Loader=yaml.SafeLoader)
        if not "drivers" in self.drivers.keys():
            raise rclpy.exceptions.InvalidParameterValueException("drivers_file must be a path to a yaml dictionary containing the key \"drivers\"")
        print(self.drivers)

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = InitializerUDPRecevier()
    serviceclient : rclpy.client.Client = node.create_client(lifecycle_msgs.srv.ChangeState, "/raw_udp_receiver_node/change_state")
    serviceclient.wait_for_service()
    req : lifecycle_msgs.srv.ChangeState.Request = lifecycle_msgs.srv.ChangeState.Request()

    req.transition.label="configure"
    req.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    future = serviceclient.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response : lifecycle_msgs.srv.ChangeState.Response = future.result()
    if not response.success:
        node.get_logger().error("Unable to configure the udp receiver")
        rclpy.shutdown()
        exit(-1)
    req.transition.label="activate"
    req.transition.id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
    future = serviceclient.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if not response.success:
        node.get_logger().error("Unable to activate the udp receiver")
        rclpy.shutdown()
        exit(-1)
    if len(node.drivers["drivers"])==0:
        exit(0)
    
    load_node_serviceclient : rclpy.client.Client = node.create_client(composition_interfaces.srv.LoadNode, "game_udp_container/_container/load_node")
    load_node_serviceclient.wait_for_service()
    node.get_logger().info("Got the load node service")
    for (i,d_) in enumerate(node.drivers["drivers"]):
        driverdict : dict = d_
        node.get_logger().info("Loading measurement publisher %s" % (str(driverdict),))
        drivername : str = driverdict["name"]
        driverindex : int = driverdict["index"]

        load_node_req : composition_interfaces.srv.LoadNode.Request = composition_interfaces.srv.LoadNode.Request()
        load_node_req.node_name="measurement_publisher"#_%s" % (drivername, )
        load_node_req.node_namespace = "/%s" % (drivername, )
        load_node_req.package_name="deepracing_rclcpp"
        load_node_req.plugin_name="deepracing::composable_nodes::MeasurementPublisher"

        index_param : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
        index_param.name = "index"
        index_param.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER
        index_param.value.integer_value = driverindex
        load_node_req.parameters.append(index_param)

        carname_param : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
        carname_param.name = "carname"
        carname_param.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING
        carname_param.value.string_value = drivername
        load_node_req.parameters.append(carname_param)

        centroid_to_base_param : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
        centroid_to_base_param.name = "centroid_to_base_translation"
        centroid_to_base_param.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_DOUBLE_ARRAY
        centroid_to_base_param.value.double_array_value = list(driverdict["centroid_to_base_translation"])
        load_node_req.parameters.append(centroid_to_base_param)

        with_ekf_param : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
        with_ekf_param.name = "with_ekf"
        with_ekf_param.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_BOOL
        with_ekf_param.value.bool_value = driverdict["with_ekf"]
        load_node_req.parameters.append(with_ekf_param)


        intra_process_comm_param : rcl_interfaces.msg.Parameter = rcl_interfaces.msg.Parameter()
        intra_process_comm_param.name = "use_intra_process_comms"
        intra_process_comm_param.value.type = rcl_interfaces.msg.ParameterType.PARAMETER_BOOL
        intra_process_comm_param.value.bool_value = True
        load_node_req.extra_arguments.append(intra_process_comm_param)
        
        load_node_req.log_level = 20

        future = load_node_serviceclient.call_async(load_node_req)
        rclpy.spin_until_future_complete(node, future)
        response : lifecycle_msgs.srv.ChangeState.Response = future.result()
        if not response.success:
            node.get_logger().error("Unable to add a driver to the ComposableNode container: %s" % (str(driverdict),))
            exit(-1)
    exit(0)

if __name__ == '__main__':
    main()