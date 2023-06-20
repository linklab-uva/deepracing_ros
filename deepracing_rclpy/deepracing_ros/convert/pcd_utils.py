
from typing import Tuple, Union
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2
import deepracing.path_utils

_DATATYPES_INVERSE : dict = dict()
for k in sensor_msgs_py.point_cloud2._DATATYPES.keys():
    _DATATYPES_INVERSE[sensor_msgs_py.point_cloud2._DATATYPES[k]] = k

def structuredArrayAsPointcloud2(structured_array : np.ndarray, shape : tuple = None, header = Header()) -> PointCloud2:
    rtn : PointCloud2 = PointCloud2(is_dense=True, is_bigendian=False, header=header)
    if shape is not None:
        rtn.height = shape[0]
        rtn.width = shape[1]
    else:
        rtn.height = 1
        rtn.width = structured_array.shape[0]
    numpytype : np.dtype = structured_array.dtype
    rtn.point_step = numpytype.itemsize
    rtn.row_step = rtn.point_step*rtn.width
    for name in numpytype.names:
        fieldtype, offset = numpytype.fields[name]
        pf : PointField = PointField()
        pf.name = name
        pf.offset = offset
        subtype, shape = fieldtype.subdtype
        pf.count = shape[0]
        pf.datatype = _DATATYPES_INVERSE[subtype]
        rtn.fields.append(pf)
    rtn.data = structured_array.tobytes()
    return rtn

def pcdAsPointCloud2(filepath : str, align = False, with_numpy_arrays = True, header = Header()) -> PointCloud2:
    numpytype, structured_array, height, width = deepracing.path_utils.loadPCD(filepath, align=align)
    pc2 = structuredArrayAsPointcloud2(numpytype, structured_array, (height, width), header=header)
    if with_numpy_arrays:
        return pc2, numpytype, structured_array
    else:
        return pc2
    
