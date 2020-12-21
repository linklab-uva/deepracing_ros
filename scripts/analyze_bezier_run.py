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

from deepracing_msgs.msg import BezierCurve, TimestampedPacketMotionData, PacketMotionData, CarMotionData, PacketHeader
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from sensor_msgs.msg import CompressedImage

import torch, torchvision

import deepracing, deepracing.pose_utils, deepracing.raceline_utils, deepracing.evaluation_utils, deepracing_models

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
from sympy import Point as SPPoint, Polygon as SPPolygon, pi
from shapely.geometry import Point as ShapelyPoint, MultiPoint#, Point2d as ShapelyPoint2d
from shapely.geometry.polygon import Polygon
from shapely.geometry import LinearRing
import shutil
import time
import cv2
import yaml 
import deepracing_ros.utils.rosbag_utils as rosbag_utils

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
parser.add_argument("--save_all_figures", action="store_true",  help="Save a plot for every bezier curve. Even ones with no track violation.")
# parser.add_argument("--trackfiledir", help="Path to the directory containing the raceline json files. Default is environment variable F1_TRACK_DIR",  type=str, default=os.environ.get("F1_TRACK_DIR",default=None))

args = parser.parse_args()

argdict = dict(vars(args))
save_all_figures = argdict["save_all_figures"]
# trackfiledir = argdict["trackfiledir"]
# if trackfiledir is None:
#     raise ValueError("Must either specify --trackfiledir or set environment variable F1_TRACK_DIR")

bag_dir = argdict["bag_dir"]
if bag_dir[-2:] in {"\\\\","//"}:
    bag_dir = bag_dir[0:-2]
elif bag_dir[-1] in {"\\","/"}:
    bag_dir = bag_dir[0:-1]
topic_types, type_map, reader = rosbag_utils.open_bagfile(bag_dir)
with open(os.path.join(bag_dir,"metadata.yaml"),"r") as f:
    metadata_dict = yaml.load(f,Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
topic_count_dict = {entry["topic_metadata"]["name"] : entry["message_count"] for entry in metadata_dict["topics_with_message_count"]}
topic_counts = np.array( list(topic_count_dict.values()) ) 

idx = 0
total_msgs = np.sum( topic_counts )
msg_dict = {key : [] for key in topic_count_dict.keys()}
print("Loading data from bag")
for idx in tqdm(iterable=range(total_msgs)):
    if(reader.has_next()):
        (topic, data, t) = reader.read_next()
        msg_type = type_map[topic]
        msg_type_full = get_message(msg_type)
        msg = deserialize_message(data, msg_type_full)
        msg_dict[topic].append(msg)
session_packet_msgs = sorted(msg_dict["/cropped_publisher/session_data"], key=poseKey)
tracknum = session_packet_msgs[-1].udp_packet.track_id
trackname = deepracing.trackNames[tracknum]
innerboundfile = deepracing.searchForFile(trackname+"_innerlimit.json", [os.curdir] + os.getenv("F1_TRACK_DIRS").split(os.pathsep) )
outerboundfile = deepracing.searchForFile(trackname+"_outerlimit.json", [os.curdir] + os.getenv("F1_TRACK_DIRS").split(os.pathsep) )
rinner, innerboundary = deepracing.raceline_utils.loadBoundary(innerboundfile)
innerboundary_kdtree = KDTree(innerboundary[0:3].cpu().numpy().copy().transpose())
innerboundary_poly = Polygon(shell=innerboundary[[0,2]].cpu().numpy().copy().transpose().tolist())
assert(innerboundary_poly.is_valid)
router, outerboundary = deepracing.raceline_utils.loadBoundary(outerboundfile)
outerboundary_kdtree = KDTree(outerboundary[0:3].cpu().numpy().copy().transpose())
outerboundary_poly = Polygon(shell=outerboundary[[0,2]].cpu().numpy().copy().transpose().tolist())
assert(outerboundary_poly.is_valid)






lapdata_msgs = sorted(msg_dict["/cropped_publisher/lap_data"], key=poseKey)
player_car_idx = lapdata_msgs[-1].udp_packet.header.player_car_index
lapdata_header_timestamps = [rclpy.time.Time.from_msg(msg.header.stamp) for msg in lapdata_msgs]
lapdata_timestamps = np.array([t.nanoseconds/1E9 for t in lapdata_header_timestamps])
lapnumbers = np.array([msg.udp_packet.lap_data[player_car_idx].current_lap_num for msg in lapdata_msgs])
print(lapdata_msgs[0].udp_packet.lap_data[player_car_idx])
print(lapdata_msgs[-1].udp_packet.lap_data[player_car_idx])
idxend = np.argmax(np.diff(lapnumbers))+1
timestampend = rclpy.time.Time.from_msg(lapdata_msgs[idxend].header.stamp)
print(timestampend)


pose_msgs = [p for p in sorted(msg_dict["/ego_vehicle/pose"], key=poseKey) if rclpy.time.Time.from_msg(p.header.stamp)<=timestampend]
pose_header_timestamps = [rclpy.time.Time.from_msg(msg.header.stamp) for msg in pose_msgs]
pose_timestamps = np.array([t.nanoseconds/1E9 for t in pose_header_timestamps])

twist_msgs = [v for v in sorted(msg_dict["/ego_vehicle/velocity"], key=poseKey) if rclpy.time.Time.from_msg(v.header.stamp)<=timestampend]
twist_header_timestamps = [rclpy.time.Time.from_msg(msg.header.stamp) for msg in twist_msgs]
twist_timestamps = np.array([t.nanoseconds/1E9 for t in twist_header_timestamps])

image_msgs = sorted(msg_dict["/full_publisher/images/compressed"], key=imgKey)
image_header_timestamps = [rclpy.time.Time.from_msg(msg.header.stamp) for msg in image_msgs]
image_timestamps = np.array([t.nanoseconds/1E9 for t in image_header_timestamps])

predicted_path_msgs = sorted(msg_dict["/predicted_paths"], key=pathsKey)
path_header_timestamps = [rclpy.time.Time.from_msg(msg.ego_pose.header.stamp) for msg in predicted_path_msgs]
path_timestamps = np.array([t.nanoseconds/1E9 for t in path_header_timestamps])




t0 = path_timestamps[0]
path_timestamps = path_timestamps  - t0
image_timestamps = image_timestamps  - t0
pose_timestamps = pose_timestamps  - t0
lapdata_timestamps = lapdata_timestamps  - t0
twist_timestamps = twist_timestamps  - t0

print("Extracted %d path comparisons" % ( len(predicted_path_msgs), ) )
print("Extracted %d images" % ( len(image_msgs), ) )


fig = plt.figure()


ib_recon = np.array(innerboundary_poly.exterior.xy).transpose()
ob_recon = np.array(outerboundary_poly.exterior.xy).transpose()
all_recon = np.row_stack([ib_recon, ob_recon])
allx = all_recon[:,0]
allz = all_recon[:,1]
minx = float(np.min(allx))-5.0
maxx = float(np.max(allx))+5.0
minz = float(np.min(allz))-5.0
maxz = float(np.max(allz))+5.0

plt.xlim(maxx,minx)
# plt.ylim(maxz,minz)

plt.plot(ib_recon[:,0],ib_recon[:,1], label="Inner Boundary of Track")

plt.plot(ob_recon[:,0],ob_recon[:,1], label="Outer Boundary of Track")
plt.legend()
plt.show()


bridge = cv_bridge.CvBridge()
positions = np.array( [ [p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in pose_msgs ] )
position_spline : BSpline = make_interp_spline(pose_timestamps, positions) 
quats = np.array( [ [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in pose_msgs ] )
rotations = Rotation.from_quat(quats)
rotation_spline : RotationSpline = RotationSpline(pose_timestamps, rotations) 


results = deepracing.evaluation_utils.lapMetrics(positions, pose_timestamps, innerboundary_poly, outerboundary_poly)
results["laptime"] = lapdata_msgs[-1].udp_packet.lap_data[player_car_idx].last_lap_time

bag_base = os.path.basename(bag_dir)
bag_parent = os.path.abspath( os.path.join(bag_dir, os.pardir) )


with open(os.path.join(bag_parent, bag_base+"_results.json"), "w") as f:
    json.dump(results, f, indent=2)









if predicted_path_msgs[0].optimization_time<=0.0:
    exit(0)

Mpaths = deepracing_models.math_utils.bezierM(torch.linspace(0,1,250, dtype=torch.float32).unsqueeze(0), len(predicted_path_msgs[0].curves[0].control_points_lateral)-1)[0]
Mboundaries = deepracing_models.math_utils.bezierM(torch.linspace(0,1,250, dtype=torch.float32).unsqueeze(0), len(predicted_path_msgs[0].inner_boundary_curve.control_points_lateral)-1)[0]#.repeat(2,1,1)
Nsamp = 90
figure_dir = os.path.join(bag_parent, bag_base+"_figures")
if os.path.isdir(figure_dir):
    shutil.rmtree(figure_dir)
time.sleep(1.0)
os.makedirs(figure_dir)
imwriter = None
print("Analyzing bezier curves")
try:
    for i, paths_msg in tqdm(enumerate(predicted_path_msgs), total=len(predicted_path_msgs)):
        t = path_timestamps[i]
        curves = torch.stack([deepracing_ros.convert.fromBezierCurveMsg(msg, dtype=Mpaths.dtype, device=Mpaths.device) for msg in paths_msg.curves], dim=0)
        boundarycurves = torch.stack([deepracing_ros.convert.fromBezierCurveMsg(paths_msg.outer_boundary_curve, dtype=Mpaths.dtype, device=Mpaths.device), deepracing_ros.convert.fromBezierCurveMsg(paths_msg.inner_boundary_curve, dtype=Mpaths.dtype, device=Mpaths.device)], dim=0)    
       # print(curves)
        #print(curves.shape)
        if not (torch.all(curves==curves)):
            curves = curves[[0,-1]] 
        with torch.no_grad():
            curvepoints = torch.matmul(Mpaths, curves)
            boundarypoints = torch.matmul(Mboundaries, boundarycurves)
            ob_distance_matrix = torch.cdist(curvepoints[-1], boundarypoints[0])
            ob_closest_point_idx = torch.argmin(ob_distance_matrix,dim=1)[-1].item()
            
            ib_distance_matrix = torch.cdist(curvepoints[-1], boundarypoints[1])
            ib_closest_point_idx = torch.argmin(ib_distance_matrix,dim=1)[-1].item()
            boundarypoints = boundarypoints[:,0:(max(ob_closest_point_idx, ib_closest_point_idx)+20)%Mboundaries.shape[0]]
            
        
       # print(curvepoints.shape)
        network_predictions = curvepoints[0].cpu().numpy().copy()
        final_output = curvepoints[-1].cpu().numpy().copy()
        outer_boundary = boundarypoints[0].cpu().numpy().copy()
        inner_boundary = boundarypoints[1].cpu().numpy().copy()
        carposition : np.ndarray = position_spline(t)
        carrotation : Rotation = rotation_spline(t)
        homogenous_transform = torch.from_numpy(deepracing.pose_utils.toHomogenousTransform(carposition, carrotation.as_quat()).copy()).type(Mpaths.dtype).to(Mpaths.device)
        homogenous_transform_inv = torch.inverse(homogenous_transform)
    
        image_idx = bisect.bisect(image_timestamps, t)
        imnp = bridge.compressed_imgmsg_to_cv2(image_msgs[image_idx],desired_encoding="rgb8")

        boundaryx = boundarypoints[:,:,0]
        boundaryz = boundarypoints[:,:,1]
        curvex = curvepoints[:,:,0]
        curvez = curvepoints[:,:,1]

        minx = min(torch.min(curvex).item(), torch.min(boundaryx).item()) - 2.0
        maxx = max(torch.max(curvex).item(), torch.max(boundaryx).item()) + 2.0
        minz = min(torch.min(curvez).item(), torch.min(boundaryz).item()) - 5.0
        maxz = max(torch.max(curvez).item(), torch.max(boundaryz).item()) + 5.0

        fig, (axim, axplot) = plt.subplots(nrows=1, ncols=2)#, figsize=(8,6))
        axim.imshow(imnp)
        axplot.set_xlim(maxx, minx)
        axplot.set_ylim(minz, maxz)

        axplot.scatter(inner_boundary[:,0], inner_boundary[:,1], label="Track Boundaries", marker="o", s=5.0*np.ones_like(inner_boundary[:,1]), c="black")

        axplot.plot(network_predictions[:,0], network_predictions[:,1], label="Initial Network Prediction", c="red")
        axplot.plot(final_output[:,0], final_output[:,1], label="Optimized Output", c="blue")
        figprefix = "figure_%d"
        if curves.shape[0]>2:
            figprefix = "FIGURE_%d"
            intermediate_points = curvepoints[1:-1].cpu().numpy().copy()
            for j in range(intermediate_points.shape[0]):
                axplot.plot(intermediate_points[j,:,0], intermediate_points[j,:,1], '--', label="Optimization step %d" %(j+1,))
        plt.legend()
        axplot.scatter(outer_boundary[:,0], outer_boundary[:,1], marker="o", s=5.0*np.ones_like(outer_boundary[:,1]), c="black")
        
        plt.savefig(os.path.join(figure_dir,(figprefix%(i+1,))+".svg"))
        plt.savefig(os.path.join(figure_dir,(figprefix%(i+1,))+".pdf"))
        plt.savefig(os.path.join(figure_dir,(figprefix%(i+1,))+".png"))
        plt.close()
       # plt.show()
        

        # plt.savefig(os.path.join(figure_dir,"curve_%d.svg" % (i,)), format="svg")
except KeyboardInterrupt as e:
    print("Okie Dokie")
    #imwriter.release()
# rotationmats = np.array( [ Rotation.as_matrix(r) for r in rotations] )
# homogenous_transforms = np.tile(np.eye(4), (len(rotations),1,1) )
# homogenous_transforms[:,0:3,0:3] = rotationmats
# homogenous_transforms[:,0:3,3] = positions


#rotation_matrices = np.array( [    ] for msg in  player_motion_data  )
# windowname = "image"
# cv2.namedWindow(windowname, cv2.WINDOW_AUTOSIZE)
# for i in range(images_np.shape[0]-1):
#     imnp = images_np[i]
#     cv2.imshow(windowname, imnp)
#     cv2.waitKey(int(round(1000.0*(image_timestamps[i+1] - image_timestamps[i]))))
# cv2.imshow(windowname, images_np[-1])
# cv2.waitKey(0)


    
# parser.add_argument("inner_track", type=str,  help="Json file for the inner boundaries of the track.")
# parser.add_argument("outer_track", type=str,  help="Json file for the outer boundaries of the track.")