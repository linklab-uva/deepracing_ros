from requests import head
import deepracing
import deepracing_msgs.msg
import deepracing_ros
import deepracing_ros.utils
import deepracing_ros.convert

import rclpy
import rclpy.node
import rclpy.subscription, rclpy.publisher, rclpy.executors
from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
import geometry_msgs.msg
from rclpy.time import Time
from rclpy.duration import Duration
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import tf2_ros
import sensor_msgs_py.point_cloud2
import ros2_numpy



import math
import os
import json
import numpy as np
from typing import List
from scipy.spatial.transform import Rotation
import scipy.interpolate
import rcl_interfaces.msg
import threading
class BoundaryPubNode(rclpy.node.Node):
    def __init__(self):
        super(BoundaryPubNode, self).__init__('boundary_pub_node')
        search_dirs_descriptor : rcl_interfaces.msg.ParameterDescriptor = rcl_interfaces.msg.ParameterDescriptor()
        search_dirs_descriptor.name="track_search_dirs"
        search_dirs_descriptor.type=rcl_interfaces.msg.ParameterType.PARAMETER_STRING_ARRAY
        
        search_dirs_param : rclpy.Parameter = self.declare_parameter(search_dirs_descriptor.name, value=None, descriptor=search_dirs_descriptor)
        search_dirs_val = search_dirs_param.get_parameter_value().string_array_value
        if search_dirs_val is None:
            self.search_dirs = []
        else:
            self.search_dirs = search_dirs_val
        try:
            self.search_dirs.append(os.path.join(get_package_share_directory("deepracing_launch"), "maps"))
            #i'll add this package eventually
            self.search_dirs.append(os.path.join(get_package_share_directory("deepracing_maps"), "maps"))
        except PackageNotFoundError as ex:
            pass
        searchdirs_string : str = ""
        for searchdir in self.search_dirs:
            searchdirs_string = searchdirs_string+searchdir+","
        self.get_logger().info("Looking for track files in %s." % (searchdirs_string[0:-1]))
        self.inner_boundary_pc2_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "/inner_boundary/pc2", 1)
        self.outer_boundary_pc2_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "/outer_boundary/pc2", 1)
        self.racline_pc2_pub : rclpy.publisher.Publisher = self.create_publisher(sensor_msgs.msg.PointCloud2, "/optimal_raceline/pc2", 1)
        self.session_data_sub : rclpy.subscription.Subscription = self.create_subscription(deepracing_msgs.msg.TimestampedPacketSessionData, "session_data", self.sessionDataCB, 1)
        self.trackmap : deepracing.TrackMap = None

        self.semaphore : threading.Semaphore = threading.Semaphore()


    def sessionDataCB(self, session_data : deepracing_msgs.msg.TimestampedPacketSessionData):
        # self.get_logger().info("Got some session data")
        idx = session_data.udp_packet.track_id
        trackname = deepracing.trackNames[idx]
        if (self.trackmap is None) or (not (self.trackmap.name==trackname)):
            self.get_logger().info("Searching for trackmap of new track name: %s" % (trackname,))
            if not self.semaphore.acquire(timeout=0.5):
                self.get_logger().error("Could not acquire semaphore")
                return
            try:
                self.trackmap : deepracing.TrackMap = deepracing.searchForTrackmap(trackname, self.search_dirs, align=True)
            except Exception as e:
                self.get_logger().error("Error when loading trackmap: %s" % (str(e),))
                self.semaphore.release()
                return
            self.semaphore.release()
            if self.trackmap is None:
                self.get_logger().error("Could not find trackmap of new track name: %s" % (trackname,))

    def timerCB(self):
        now = self.get_clock().now()
        if self.trackmap is None:
            self.get_logger().error("No trackmap loaded yet, not publishing")
            return
        if not self.semaphore.acquire(timeout=0.5):
            self.get_logger().error("Could not acquire semaphore")
            return
        try:
            innerbound_dict : dict = self.trackmap.linemap["inner_boundary"]
            innerbound_array : np.ndarray = innerbound_dict["line"].copy()
            outerbound_dict : dict = self.trackmap.linemap["outer_boundary"]
            outerbound_array : np.ndarray = outerbound_dict["line"].copy()
            raceline_dict : dict = self.trackmap.linemap["raceline"]
            raceline_array : np.ndarray = raceline_dict["line"].copy()
        except Exception as e:
            self.get_logger().error("Error when unpacking trackmap: %s" % (str(e),))
            self.semaphore.release()
            return
        self.semaphore.release()
        header = std_msgs.msg.Header(stamp=now.to_msg(), frame_id="map")
        ib_msg : sensor_msgs.msg.PointCloud2 = ros2_numpy.msgify(sensor_msgs.msg.PointCloud2, innerbound_array)
        ob_msg : sensor_msgs.msg.PointCloud2 = ros2_numpy.msgify(sensor_msgs.msg.PointCloud2, outerbound_array)
        rl_msg : sensor_msgs.msg.PointCloud2 = ros2_numpy.msgify(sensor_msgs.msg.PointCloud2, raceline_array)
        ib_msg.header = header
        ob_msg.header = header
        rl_msg.header = header
        self.inner_boundary_pc2_pub.publish(ib_msg)
        self.outer_boundary_pc2_pub.publish(ob_msg)
        self.racline_pc2_pub.publish(rl_msg)
    


def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.initialize()
    node = BoundaryPubNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
    executor : rclpy.executors.MultiThreadedExecutor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    node.create_timer(2.0, node.timerCB)
    executor.spin()