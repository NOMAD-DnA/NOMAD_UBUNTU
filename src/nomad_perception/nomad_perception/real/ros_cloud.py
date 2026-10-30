"""Explicit XYZ float32 ROS transport for the bridge and mapper."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def cloud_message(points, ns, frame):
    m = PointCloud2(); m.header = Header(); m.header.frame_id = frame
    m.header.stamp.sec = int(ns)//10**9; m.header.stamp.nanosec = int(ns)%10**9
    m.height = 1; m.width = len(points)
    m.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                for i, k in enumerate(['x', 'y', 'z'])]
    m.is_bigendian = False; m.point_step = 12; m.row_step = 12*len(points)
    m.is_dense = True; m.data = np.asarray(points, dtype='<f4').tobytes()
    return m


def xyz_points(m):
    fields = [(f.name, f.offset, f.datatype, f.count) for f in m.fields]
    expected = [(k, i*4, PointField.FLOAT32, 1) for i, k in enumerate(['x', 'y', 'z'])]
    if (m.is_bigendian or m.height != 1 or m.point_step != 12 or
        m.row_step != 12*m.width or len(m.data) != m.row_step or fields != expected):
        raise ValueError('Expected bridge XYZ float32 cloud layout')
    return np.frombuffer(bytes(m.data), dtype='<f4').reshape(-1, 3).copy()
