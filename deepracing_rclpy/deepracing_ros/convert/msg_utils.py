import deepracing_msgs.msg as drmsgs # BezierCurve, TimestampedPacketMotionData, PacketMotionData, CarMotionData, PacketHeader
import geometry_msgs.msg as geo_msgs#  Point, PointStamped, Vector3, Vector3Stamped
import tf2_msgs.msg as tf2_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
import numpy.linalg as la
import scipy.spatial.transform
import math
import struct
from typing import List
from scipy.spatial.transform import Rotation as Rot
import deepracing
import torch
# _DATATYPES = {
# PointField.INT8    : ('b', 1),\
# PointField.UINT8  : ('B', 1),\
# PointField.INT16   : ('h', 2),\
# PointField.UINT16 : ('H', 2),\
# PointField.INT32  : ('i', 4),\
# PointField.UINT3  : ('I', 4),\
# PointField.FLOAT32 : ('f', 4),\
# PointField.FLOAT64 : ('d', 8)
# }
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)
def _get_struct_fmt(is_bigendian, fields, field_names=None):

   fmt = '>' if is_bigendian else '<'

   offset = 0
   for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
      if offset < field.offset:
         fmt += 'x' * (field.offset - offset)
         offset = field.offset
      if field.datatype not in _DATATYPES:
         print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
      else:
         datatype_fmt, datatype_length = _DATATYPES[field.datatype]
         fmt    += field.count * datatype_fmt
         offset += field.count * datatype_length

   return fmt
def pointCloud2ToNumpy(cloud: PointCloud2, field_names=None, skip_nans=False, uvs=[]):
   """
   Read points from a L{sensor_msgs.PointCloud2} message.

   @param cloud: The point cloud to read from.
   @type  cloud: L{sensor_msgs.PointCloud2}
   @param field_names: The names of fields to read. If None, read all fields. [default: None]
   @type  field_names: iterable
   @param skip_nans: If True, then don't return any point with a NaN value.
   @type  skip_nans: bool [default: False]
   @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
   @type  uvs: iterable
   @return: Generator which yields a list of values for each point.
   @rtype:  generator
   """
   assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
   fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
   width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
   unpack_from = struct.Struct(fmt).unpack_from

   if skip_nans:
      if uvs:
         for u, v in uvs:
               p = unpack_from(data, (row_step * v) + (point_step * u))
               has_nan = False
               for pv in p:
                  if isnan(pv):
                     has_nan = True
                     break
               if not has_nan:
                  yield p
      else:
         for v in range(height):
               offset = row_step * v
               for u in range(width):
                  p = unpack_from(data, offset)
                  has_nan = False
                  for pv in p:
                     if isnan(pv):
                           has_nan = True
                           break
                  if not has_nan:
                     yield p
                  offset += point_step
   else:
      if uvs:
         for u, v in uvs:
               yield unpack_from(data, (row_step * v) + (point_step * u))
      else:
         for v in range(height):
               offset = row_step * v
               for u in range(width):
                  yield unpack_from(data, offset)
                  offset += point_step

def arrayToPointCloud2(pointsarray : [torch.Tensor, np.ndarray], field_names : List[str], header : Header, is_bigendian = False):
   if isinstance(pointsarray, torch.Tensor):
      points = pointsarray.detach().cpu().numpy()
   elif isinstance(pointsarray, np.ndarray):
      points = pointsarray
   else:
      raise TypeError("arrayToPointCloud2 only supports torch.Tensor and np.ndarray as input. Got unknown type: %s" %(str(type(pointsarray)),))

   numfields = len(field_names)
   assert(numfields==points.shape[1])
   pc2out = PointCloud2(header=header, is_bigendian = is_bigendian, is_dense = True, width = points.shape[0], height = 1)
   if points.dtype==np.float64:
      bytesperfield = 8
      dtypemsg = PointField.FLOAT64
   elif points.dtype==np.float32:
      bytesperfield = 4
      dtypemsg = PointField.FLOAT32
   else:
      raise ValueError("Only float32 and float64 arrays are supported")
   pc2out.point_step = bytesperfield*numfields
   pc2out.row_step=pc2out.point_step*pc2out.width
   pc2out.fields=[PointField(name=name, offset=i*bytesperfield, count=1, datatype=dtypemsg) for (i,name) in enumerate(field_names)]
   pc2out.data = points.flatten().tobytes()
   return pc2out

def extractPosition(packet : drmsgs.PacketMotionData , car_index = None) -> np.ndarray:
   if car_index is None:
      idx = packet.header.player_car_index
   else:
      idx = car_index
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]
   position = np.array( (motion_data.world_position.point.x, motion_data.world_position.point.y, motion_data.world_position.point.z), dtype=np.float64)
   return position 

def extractOrientation(packet : drmsgs.PacketMotionData, car_index = None) -> scipy.spatial.transform.Rotation:
   if car_index is None:
      idx = packet.header.player_car_index
   else:
      idx = car_index
   motion_data : drmsgs.CarMotionData = packet.car_motion_data[idx]

   rightdir : geo_msgs.Vector3 = motion_data.world_right_dir.vector
   forwarddir : geo_msgs.Vector3 = motion_data.world_forward_dir.vector

   rightvector = np.array((rightdir.x, rightdir.y, rightdir.z), dtype=np.float64)
   rightvector = rightvector/la.norm(rightvector)

   forwardvector = np.array((forwarddir.x, forwarddir.y, forwarddir.z), dtype=np.float64)
   forwardvector = forwardvector/la.norm(forwardvector)

   upvector = np.cross(rightvector,forwardvector)
   upvector = upvector/la.norm(upvector)
   rotationmat = np.column_stack([-rightvector,upvector,forwardvector])
   return scipy.spatial.transform.Rotation.from_matrix(rotationmat)

def extractPose(packet : drmsgs.PacketMotionData, car_index = None):
   if car_index is None:
      idx = packet.header.player_car_index
   else:
      idx = car_index
   p = extractPosition(packet, car_index=idx)
   q = extractOrientation(packet, car_index=idx)
   return ( p, q )

def toBezierCurveMsg(control_points, header: Header, covars = None):
   ptsnp = control_points.detach().cpu().numpy()
   if covars is not None:
      assert(ptsnp.shape[0]==covars.shape[0])
      covarmatsnp = covars.view(covars.shape[0],9).detach().cpu().numpy()
   rtn : drmsgs.BezierCurve = drmsgs.BezierCurve(header=header)
   for i in range(ptsnp.shape[0]):
      rtn.control_points.append(geo_msgs.Point(x=float(ptsnp[i,0]), y=float(ptsnp[i,1]), z=float(ptsnp[i,2])))
      if covars is not None: 
         covar : drmsgs.CovariancePoint = drmsgs.CovariancePoint()
         covar.covariance = covarmatsnp[i].tolist()
         rtn.control_point_covariances.append(covar)
   return rtn
def fromBezierCurveMsg(curve_msg : drmsgs.BezierCurve, dtype=torch.float32, device=torch.device("cpu")):
   ptsnp = np.array([[p.x, p.y, p.z ] for p in curve_msg.control_points ], copy=False)
   covarslength = len(curve_msg.control_point_covariances)
   if covarslength>0:
      if not (covarslength==ptsnp.shape[0]):
         raise ValueError("Tried to unpack a bezier curve with %d control points, but with %d covariance matrices. A BezierCurve message must either have 0 covariances or the the same as the number of control points." % (ptsnp.shape[0], covarslength))
      covariances = torch.stack([torch.as_tensor(curve_msg.control_point_covariances[i].covariance, dtype=dtype, device=device) for i in range(covarslength)], dim=0).view(covarslength,3,3)
   else:
      covariances = None
   return torch.as_tensor(ptsnp.copy(), device=device, dtype=dtype), covariances
def transformMsgToTorch(transform_msg: geo_msgs.Transform, dtype=torch.float32, device=torch.device("cpu")):
   rtn = torch.eye(4, dtype=dtype, device=device, requires_grad=False)
   rtn[0:3,0:3] = torch.as_tensor(Rot.from_quat(np.array([transform_msg.rotation.x, transform_msg.rotation.y, transform_msg.rotation.z, transform_msg.rotation.w], copy=False)).as_matrix().copy(), dtype=dtype, device=device)
   rtn[0:3,3] = torch.as_tensor(np.array([transform_msg.translation.x, transform_msg.translation.y, transform_msg.translation.z]), dtype=dtype, device=device)
   return rtn
def poseMsgToTorch(pose_msg: geo_msgs.Pose, dtype=torch.float32, device=torch.device("cpu")):
   rtn = torch.eye(4, dtype=dtype, device=device, requires_grad=False)
   rtn[0:3,0:3] = torch.as_tensor(Rot.from_quat(np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w], copy=False)).as_matrix().copy(), dtype=dtype, device=device)
   rtn[0:3,3] = torch.as_tensor(np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]), dtype=dtype, device=device)
   return rtn
def pointMsgToTorch(point_msg: geo_msgs.Point, dtype=torch.float32, device=torch.device("cpu")):
   return torch.as_tensor([point_msg.x, point_msg.y, point_msg.z], dtype=dtype, device=device)

def vectorMsgToTorch(vector_msg: geo_msgs.Vector3, dtype=torch.float32, device=torch.device("cpu")):
   return torch.as_tensor([vector_msg.x, vector_msg.y, vector_msg.z], dtype=dtype, device=device)