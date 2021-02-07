import rclpy, rclpy.time, rclpy.duration, deepracing_ros
import rpyutils
with rpyutils.add_dll_directories_from_env("PATH"):
    import cv_bridge
    import rosbag2_py
import argparse
import typing
from typing import List

from tqdm import tqdm as tqdm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from deepracing_msgs.msg import TimestampedPacketMotionData
from deepracing_msgs.msg import BezierCurve, PacketMotionData, CarMotionData, PacketHeader, LapData, CarTelemetryData, CarMotionData, PacketSessionData, TrajComparison
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from sensor_msgs.msg import CompressedImage

import torch, torchvision

import deepracing, deepracing.pose_utils, deepracing.raceline_utils, deepracing.evaluation_utils, deepracing_models, deepracing_models.math_utils as mu

import numpy as np
import cv2

import deepracing_ros, deepracing_ros.convert
from scipy.spatial.transform import Rotation, RotationSpline
from scipy.interpolate import BSpline, make_interp_spline

import deepracing_models, deepracing_models.math_utils
import bisect
import json
import scipy
from scipy.spatial.kdtree import KDTree
import matplotlib.pyplot as plt
import os
from shapely.geometry import Point as ShapelyPoint, MultiPoint#, Point2d as ShapelyPoint2d
from shapely.geometry.polygon import Polygon
from shapely.geometry import LinearRing
import shutil
import time
import cv2
import yaml 
import deepracing_ros.utils.rosbag_utils as rosbag_utils

from ament_index_python import get_package_share_directory
from tqdm import tqdm as tqdm

def extractPosition(vectormsg):
    return np.array( [ msg.x, msg.y, msg.z ] )
def imgKey(msg):
    return rclpy.time.Time.from_msg(msg.header.stamp)
def pathsKey(msg):
    return rclpy.time.Time.from_msg(msg.ego_pose.header.stamp)
def poseKey(msg):
    return rclpy.time.Time.from_msg(msg.header.stamp)

parser = argparse.ArgumentParser(description="Look for bad predictions in a run of the bezier curve predictor")
parser.add_argument("bag_dir", type=str,  help="Bag to load")


args = parser.parse_args()
argdict : dict = vars(args)
bag_dir = os.path.abspath(argdict["bag_dir"])
if bag_dir[-1] in {"\\", "/"}:
    bag_dir = bag_dir[0:-1]



bridge = cv_bridge.CvBridge()

topic_types, type_map, reader = rosbag_utils.open_bagfile(bag_dir)
with open(os.path.join(bag_dir,"metadata.yaml"),"r") as f:
    metadata_dict = yaml.load(f,Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
topic_counts = np.array( list(topic_count_dict.values()) ) 
telemetry_data_msgs = []
lap_data_msgs = []
idx = 0
total_msgs = np.sum( topic_counts )

#{'/f1_screencaps/cropped/compressed': 'sensor_msgs/msg/CompressedImage', '/motion_data': 'deepracing_msgs/msg/TimestampedPacketMotionData', '/predicted_path': 'deepracing_msgs/msg/BezierCurve'}
print("Loading data from bag: %s" % (bag_dir,))
msg_dict = {key : [] for key in topic_count_dict.keys()}
for idx in tqdm(iterable=range(total_msgs)):
    if(reader.has_next()):
        (topic, data, t) = reader.read_next()
        msg_type = type_map[topic]
        msg_type_full = get_message(msg_type)
        msg = deserialize_message(data, msg_type_full)
        msg_dict[topic].append(msg)
topic_names = list(type_map.keys())
ego_lap_data : List[LapData] = [msg.udp_packet.lap_data[msg.udp_packet.header.player_car_index] for msg in msg_dict["/cropped_publisher/lap_data"]]
lapnums = torch.as_tensor([ld.current_lap_num for ld in ego_lap_data], dtype=torch.int32)
lapdistances = torch.as_tensor([ld.lap_distance for ld in ego_lap_data], dtype=torch.float32)
Ifinallap1 = torch.argmax(lapnums).item() - 1
Ifirstlap1 = torch.argmax((lapdistances>=0.0).byte()).item() - 1

session_packets : List[PacketSessionData] = [msg.udp_packet for msg in msg_dict["/cropped_publisher/session_data"]]

ego_telemetry : List[CarTelemetryData] = [msg.udp_packet.car_telemetry_data[msg.udp_packet.header.player_car_index] for msg in msg_dict["/cropped_publisher/telemetry_data"]]
ego_throttles = torch.as_tensor([msg.throttle for msg in ego_telemetry], dtype=torch.uint8)

gunnin_it = ego_throttles>50
whoa_there =  ~gunnin_it

ego_positions = torch.as_tensor( [ [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] for msg in msg_dict["/ego_vehicle/pose"] ] , dtype=torch.float64 )
ego_quaternions = torch.as_tensor( [ [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w] for msg in msg_dict["/ego_vehicle/pose"] ] , dtype=torch.float64 )
ego_velocities = torch.as_tensor( [ [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] for msg in msg_dict["/ego_vehicle/velocity"] ] , dtype=torch.float64 )

ego_speeds = torch.norm(ego_velocities, p=2, dim=1)

results : dict = {}
results["average_speed"] = torch.mean(ego_speeds).item()
results["throttle_ratio"] = torch.mean(gunnin_it.double()).item()


track_id = session_packets[-1].track_id
track_name = deepracing.trackNames[track_id]
search_dirs = os.getenv("F1_TRACK_DIRS","").split(os.pathsep) + [os.path.join(get_package_share_directory("f1_datalogger"), "f1_tracks")]
inner_boundary_file = deepracing.searchForFile(track_name+"_innerlimit.json", search_dirs)
outer_boundary_file = deepracing.searchForFile(track_name+"_outerlimit.json", search_dirs)
raceline_file = deepracing.searchForFile(track_name+"_racingline.json", search_dirs)

with open(inner_boundary_file, "r") as f:
    inner_boundary_dict = json.load(f)
    inner_boundary = torch.stack([ torch.as_tensor(inner_boundary_dict["x"], dtype=torch.float32), torch.as_tensor(inner_boundary_dict["y"], dtype=torch.float32), torch.as_tensor(inner_boundary_dict["z"], dtype=torch.float32) ], dim=1)

with open(outer_boundary_file, "r") as f:
    outer_boundary_dict = json.load(f)
    outer_boundary = torch.stack([ torch.as_tensor(outer_boundary_dict["x"], dtype=torch.float32), torch.as_tensor(outer_boundary_dict["y"], dtype=torch.float32), torch.as_tensor(outer_boundary_dict["z"], dtype=torch.float32) ], dim=1)

with open(raceline_file, "r") as f:
    raceline_dict = json.load(f)
    raceline = torch.stack([ torch.as_tensor(raceline_dict["x"], dtype=torch.float32), torch.as_tensor(raceline_dict["y"], dtype=torch.float32), torch.as_tensor(raceline_dict["z"], dtype=torch.float32) ], dim=1)
   

bag_dir_base = os.path.basename(bag_dir)
results_dir = os.path.join(os.path.dirname(bag_dir), bag_dir_base+"_results")
if os.path.isdir(results_dir):
    inp = "asdf"
    while inp not in {"y","n"}:
        inp = input("Overwrite results directory %s? [y\\n]" % (results_dir,)).lower()
    if inp=="y":
        shutil.rmtree(results_dir)
        time.sleep(1.0)
os.makedirs(os.path.join(results_dir, "pdf_figures"))
os.makedirs(os.path.join(results_dir, "svg_figures"))
os.makedirs(os.path.join(results_dir, "png_figures"))
    


allpoints = torch.cat([ego_positions, inner_boundary, outer_boundary, raceline], dim=0)

min_x = torch.min(allpoints[:,0])
max_x = torch.max(allpoints[:,0])

min_z = torch.min(allpoints[:,2])
max_z = torch.max(allpoints[:,2])

fig_path = plt.figure()
plt.plot(inner_boundary[:,0].numpy(), inner_boundary[:,2].numpy(), label="Track Boundaries", c="black")
plt.plot(ego_positions[:,0].numpy(), ego_positions[:,2].numpy(), label="Ego Vehicle's Path")
plt.plot(raceline[:,0].numpy(), raceline[:,2].numpy(), label="Optimal Raceline")
plt.xlim(max_x.item()+10.0, min_x.item()-10.0)
plt.legend()
plt.plot(outer_boundary[:,0].numpy(), outer_boundary[:,2].numpy(), label="Track Boundaries", c="black")
plt.savefig(os.path.join(results_dir, "path.eps"), format="eps")
plt.savefig(os.path.join(results_dir, "path.svg"), format="svg")
plt.savefig(os.path.join(results_dir, "path.png"), format="png")
plt.close()

with open(os.path.join(results_dir, "results.json"), "w") as f:
    json.dump(results, f, indent=2)

initial_curves_local = torch.as_tensor([ deepracing_ros.convert.fromBezierCurveMsg(msg.initial_curve)[0].numpy() for msg in msg_dict["/trajectory_comparisons"] ], device=torch.device("cuda:0"), dtype=torch.float64)
initial_curves_local_aug = torch.cat([initial_curves_local, torch.ones_like(initial_curves_local[:,:,0]).unsqueeze(2)], dim=2)
filtered_curves_local = torch.as_tensor([ deepracing_ros.convert.fromBezierCurveMsg(msg.filtered_curve)[0].numpy() for msg in msg_dict["/trajectory_comparisons"] ], device=torch.device("cuda:0"), dtype=torch.float64)
filtered_curves_local_aug = torch.cat([filtered_curves_local, torch.ones_like(filtered_curves_local[:,:,0]).unsqueeze(2)], dim=2)
curve_poses = torch.as_tensor([ deepracing_ros.convert.poseMsgToTorch(msg.ego_pose.pose).numpy() for msg in msg_dict["/trajectory_comparisons"] ], device=torch.device("cuda:0"), dtype=torch.float64)
curve_poses_inv = torch.inverse(curve_poses)
raceline = raceline.type(curve_poses.dtype)
inner_boundary = inner_boundary.type(curve_poses.dtype)
outer_boundary = outer_boundary.type(curve_poses.dtype)

storch = torch.linspace(0.0, 1.0, steps=240, device=initial_curves_local.device, dtype=initial_curves_local.dtype).unsqueeze(0)
bezierM = mu.bezierM(storch, initial_curves_local.shape[1]-1)
bezierMderiv = mu.bezierM(storch, initial_curves_local.shape[1]-2)
bezierM2ndderiv = mu.bezierM(storch, initial_curves_local.shape[1]-3)

dT = 1.5
initial_curves_local_points = torch.matmul(bezierM.expand(initial_curves_local.shape[0],-1,-1), initial_curves_local).cpu()
_, initial_curves_vs= mu.bezierDerivative(initial_curves_local, M=bezierMderiv.expand(initial_curves_local.shape[0],-1,-1), order=1)
initial_curves_velocity = initial_curves_vs/dT
initial_curves_speeds = torch.norm(initial_curves_velocity, p=2, dim=2)
initial_curves_average_speeds = torch.mean(initial_curves_speeds, dim=1)
_, initial_curves_as= mu.bezierDerivative(initial_curves_local, M=bezierM2ndderiv.expand(initial_curves_local.shape[0],-1,-1), order=2)
initial_curves_acceleration = initial_curves_as/(dT**2)
initial_curves_linear_accel_norms = torch.norm(initial_curves_acceleration, p=2, dim=2)
initial_curves_avg_linear_accels = torch.mean(initial_curves_linear_accel_norms, dim=1)

filtered_curves_local_points = torch.matmul(bezierM.expand(filtered_curves_local.shape[0],-1,-1), filtered_curves_local).cpu()
_, filtered_curves_vs = mu.bezierDerivative(filtered_curves_local, M=bezierMderiv.expand(filtered_curves_local.shape[0],-1,-1), order=1)
filtered_curves_velocity = filtered_curves_vs/dT
filtered_curves_speeds = torch.norm(filtered_curves_velocity, p=2, dim=2)
filtered_curves_average_speeds = torch.mean(filtered_curves_speeds, dim=1)
_, filtered_curves_as= mu.bezierDerivative(filtered_curves_local, M=bezierM2ndderiv.expand(filtered_curves_local.shape[0],-1,-1), order=2)
filtered_curves_acceleration = filtered_curves_as/(dT**2)
filtered_curves_linear_accel_norms = torch.norm(filtered_curves_acceleration, p=2, dim=2)
filtered_curves_avg_linear_accels = torch.mean(filtered_curves_linear_accel_norms, dim=1)

speed_deltas = (filtered_curves_average_speeds - initial_curves_average_speeds)
accel_deltas = (filtered_curves_avg_linear_accels - initial_curves_avg_linear_accels)
print(torch.mean(speed_deltas))

plt.figure()
plt.title("Difference in average speed between initial curve and filtered curve")
# plt.hist(initial_curves_average_speeds.cpu().numpy(), bins=75, label="Initial Curve Speeds")
# plt.hist(filtered_curves_average_speeds.cpu().numpy(), bins=75, label="Filtered Curve Speeds")
plt.hist(speed_deltas.cpu().numpy(), bins=100)
plt.xlabel("Average Speed Difference (m/s)")
plt.savefig(os.path.join(results_dir, "speed_histogram.pdf"), format="pdf")
plt.savefig(os.path.join(results_dir, "speed_histogram.svg"), format="svg")
plt.savefig(os.path.join(results_dir, "speed_histogram.png"), format="png")
plt.close()



initial_curves_global = torch.matmul(initial_curves_local_aug, curve_poses.transpose(1,2))[:,:,0:3]
initial_curves_global_points = torch.matmul(bezierM.expand(initial_curves_global.shape[0],-1,-1), initial_curves_global).cpu()

filtered_curves_global = torch.matmul(filtered_curves_local_aug, curve_poses.transpose(1,2))[:,:,0:3]
filtered_curves_global_points = torch.matmul(bezierM.expand(filtered_curves_global.shape[0],-1,-1), filtered_curves_global).cpu()




for (i, msg) in tqdm(enumerate(list(msg_dict["/trajectory_comparisons"]))):
    comparison_msg : TrajComparison = msg
    ego_pose = curve_poses[i].cpu()
    ego_pose_inv = curve_poses_inv[i].cpu()
    initial_curve_points = initial_curves_local_points[i]
    filtered_curve_points = filtered_curves_local_points[i]
    initial_curve_points_global = initial_curves_global_points[i]
    filtered_curve_points_global = filtered_curves_global_points[i]

    dI = 20
    ibclosest, ibfurthest = (torch.argmin(torch.norm(inner_boundary - ego_pose[0:3,3], p=2, dim=1)) - dI)%inner_boundary.shape[0], (torch.argmin(torch.norm(inner_boundary - filtered_curve_points_global[-1], p=2, dim=1)) + dI)%inner_boundary.shape[0]
    obclosest, obfurthest = (torch.argmin(torch.norm(outer_boundary - ego_pose[0:3,3], p=2, dim=1)) - dI)%outer_boundary.shape[0], (torch.argmin(torch.norm(outer_boundary - filtered_curve_points_global[-1], p=2, dim=1)) + dI)%outer_boundary.shape[0]

    if ibclosest<ibfurthest:
        ibidx = torch.arange(ibclosest, ibfurthest, step=1, dtype=torch.int64)%inner_boundary.shape[0]
    else:
        ibidx = torch.arange(ibclosest, ibfurthest + inner_boundary.shape[0], step=1, dtype=torch.int64)%inner_boundary.shape[0]
    inner_boundary_global = inner_boundary[ibidx]
    inner_boundary_aug = torch.cat([inner_boundary_global, torch.ones_like(inner_boundary_global[:,0]).unsqueeze(1)], dim=1)
    inner_boundary_local = torch.matmul(inner_boundary_aug, ego_pose_inv.t())[:,0:3]


    if obclosest<obfurthest:
        obidx = torch.arange(obclosest, obfurthest, step=1, dtype=torch.int64)%outer_boundary.shape[0]
    else:
        obidx = torch.arange(obclosest, obfurthest + outer_boundary.shape[0], step=1, dtype=torch.int64)%outer_boundary.shape[0]
    outer_boundary_global = outer_boundary[obidx]
    outer_boundary_aug = torch.cat([outer_boundary_global, torch.ones_like(outer_boundary_global[:,0]).unsqueeze(1)], dim=1)
    outer_boundary_local = torch.matmul(outer_boundary_aug, ego_pose_inv.t())[:,0:3]


    fig_curve = plt.figure()
    plt.plot(initial_curve_points[:,0].cpu().numpy(), initial_curve_points[:,2].cpu().numpy(), label="Initial Curve")
    plt.plot(filtered_curve_points[:,0].cpu().numpy(), filtered_curve_points[:,2].cpu().numpy(), label="Filtered Curve")
    plt.plot(inner_boundary_local[:,0].cpu().numpy(), inner_boundary_local[:,2].cpu().numpy(), label="Track Boundaries", c="black")
    plt.legend()
    plt.plot(outer_boundary_local[:,0].cpu().numpy(), outer_boundary_local[:,2].cpu().numpy(), label="Track Boundaries", c="black")
    plt.savefig(os.path.join(results_dir, "pdf_figures", "curve_%d.pdf" %(i,)), format="pdf")
    plt.savefig(os.path.join(results_dir, "svg_figures", "curve_%d.svg" %(i,)), format="svg")
    plt.savefig(os.path.join(results_dir, "png_figures", "curve_%d.png" %(i,)), format="png")
    plt.close()







