
from typing import Tuple
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

#From the PCD spec, the valid type characters are: 
#I - represents signed types int8 (char), int16 (short), and int32 (int)
#U - represents unsigned types uint8 (unsigned char), uint16 (unsigned short), uint32 (unsigned int)
#F - represents float types

_SIGNED_INTEGER_SIZEMAP : dict[int,int] = {}
_SIGNED_INTEGER_SIZEMAP[1] = PointField.INT8
_SIGNED_INTEGER_SIZEMAP[2] = PointField.INT16
_SIGNED_INTEGER_SIZEMAP[4] = PointField.INT32

_UNSIGNED_INTEGER_SIZEMAP : dict[int,int] = {}
_UNSIGNED_INTEGER_SIZEMAP[1] = PointField.UINT8
_UNSIGNED_INTEGER_SIZEMAP[2] = PointField.UINT16
_UNSIGNED_INTEGER_SIZEMAP[4] = PointField.UINT32

_FLOATING_POINT_SIZEMAP : dict[int,int] = {}
_FLOATING_POINT_SIZEMAP[4] = PointField.FLOAT32
_FLOATING_POINT_SIZEMAP[8] = PointField.FLOAT64

_TYPEMAP : dict[str,dict[int,int]] = {}
_TYPEMAP["I"] = _SIGNED_INTEGER_SIZEMAP
_TYPEMAP["U"] = _UNSIGNED_INTEGER_SIZEMAP
_TYPEMAP["F"] = _FLOATING_POINT_SIZEMAP

# Valid PCD files have the following header keys in this specific order:
# VERSION
# FIELDS
# SIZE
# TYPE
# COUNT
# WIDTH
# HEIGHT
# VIEWPOINT
# POINTS
# DATA
_VERSION_TAG_LINE = 0
_FIELDS_TAG_LINE = 1
_SIZE_TAG_LINE = 2
_TYPE_TAG_LINE = 3
_COUNT_TAG_LINE = 4
_WIDTH_TAG_LINE = 5
_HEIGHT_TAG_LINE = 6
_VIEWPOINT_TAG_LINE = 7
_POINTS_TAG_LINE =8
_DATA_TAG_LINE = 9
_NUM_PCD_HEADER_LINES=10

#An examples of such a header for an ascii PCD

# VERSION 0.7
# FIELDS x y z distance
# SIZE 4 4 4 4
# TYPE F F F F
# COUNT 1 1 1 1
# WIDTH 2640
# HEIGHT 1
# VIEWPOINT -0.271 0.654 0.653 0.271 -109.206 2.884 463.537
# POINTS 2640
# DATA ascii

def decodePCDHeader(headerlines : list[str], align=False) -> Tuple[list[PointField], np.dtype, int]:
    fields_string = headerlines[_FIELDS_TAG_LINE].replace("FIELDS","").strip()
    fieldnames : list[str] = fields_string.split(" ")
    numfields = len(fieldnames)

    sizes_string : str = headerlines[_SIZE_TAG_LINE].replace("SIZE","").strip()
    sizes : list[int] = [int(s) for s in sizes_string.split(" ")]
    numsizes = len(sizes)

    if numsizes!=numfields:
        raise ValueError("Got FIELDS tag: %s with %d elements, but SIZE tag %s with %d elements" % (fields_string, numfields, sizes_string, numsizes))

    types_string : str = headerlines[_TYPE_TAG_LINE].replace("TYPE","").strip()
    types : list[str] = types_string.split(" ")
    numtypes = len(types)

    if numtypes!=numfields:
        raise ValueError("Got FIELDS tag: %s with %d elements, but TYPE tag %s with %d elements" % (fields_string, numfields, types_string, numtypes))

    counts_string : str = headerlines[_COUNT_TAG_LINE].replace("COUNT","").strip()
    counts : list[int] = [int(s) for s in counts_string.split(" ")]
    numcounts = len(counts)

    if numcounts!=numfields:
        raise ValueError("Got FIELDS tag: %s with %d elements, but COUNT tag %s with %d elements" % (fields_string, numfields, counts_string, numcounts))

    height_string : str = headerlines[_HEIGHT_TAG_LINE].replace("HEIGHT","").strip()
    height : int = int(height_string)

    width_string : str = headerlines[_WIDTH_TAG_LINE].replace("WIDTH","").strip()
    width : int = int(width_string)

    numpoints_string : str = headerlines[_POINTS_TAG_LINE].replace("POINTS","").strip()
    numpoints : int = int(numpoints_string)

    pointfieldsmsgs : list[PointField] = []
    numpytuples : list[Tuple] = []
    for i in range(numfields):  
        pf : PointField = PointField()
        pf.name = fieldnames[i]
        pf.count = counts[i]

        typestr = types[i]
        size = sizes[i]
        if typestr not in _TYPEMAP.keys():
            raise ValueError("Got invalid type %s for field %s, only supported types are %s" % (typestr, pf.name, str(_TYPEMAP.keys())))
        sizemap = _TYPEMAP[typestr]
        if size not in sizemap.keys():
            raise ValueError("Got invalid size %d for field %s of type %s, only supported size for type %s are %s" % (size,  pf.name, typestr, typestr, str(sizemap.keys())))
        pf.datatype = sizemap[size]
        pointfieldsmsgs.append(pf)
        numpytuples.append((pf.name, "%s%d" % (typestr.lower(), size), (pf.count,)))
    numpytype = np.dtype(numpytuples, align=align)
    numpyproxy : dict = dict(numpytype.fields)
    for field in pointfieldsmsgs:
        field.offset = (numpyproxy[field.name])[1]
    return pointfieldsmsgs, np.dtype(numpytuples), height, width, numpoints

def pcdAsPointCloud2(filepath : str, align=False) -> PointCloud2:
    rtn : PointCloud2 = PointCloud2()
    rtn.is_dense=True
    rtn.is_bigendian=False
    with open(filepath, "rb") as f:
        headerlines : list[str] = [f.readline().decode("ascii").strip() for asdf in range(_NUM_PCD_HEADER_LINES)]
        data_tag = headerlines[_DATA_TAG_LINE].replace("DATA","").strip() 
        if data_tag not in {"binary", "ascii"}:
            raise ValueError("Invalid DATA tag %s. Supported types are \"ascii\" or \"binary\"" % (data_tag,))
        fields, numpytype, height, width, numpoints = decodePCDHeader(headerlines, align=(align and (data_tag=="ascii")))
        if not (numpoints==(height*width)):
            raise ValueError("Got non-dense PCD file with height %d and width %d, but containing %d points. Number of points should equal height*width" % (height, width, numpoints))
        rtn.fields = fields
        rtn.point_step=numpytype.itemsize
        rtn.height = height
        rtn.width = width
        rtn.row_step = width*rtn.point_step
        if data_tag=="binary":
            rtn.data = f.read()
        else:
            structured_numpy_array = np.loadtxt(f, dtype=numpytype, encoding="ascii", delimiter=" ")
            rtn.data = structured_numpy_array.tobytes()
        return rtn
            
