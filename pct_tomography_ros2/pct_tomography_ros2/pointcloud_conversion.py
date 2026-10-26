import numpy as np

from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


POINT_FIELDS_XYZI = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def xyz32(header, points):
    return point_cloud2.create_cloud_xyz32(
        header, np.asarray(points, dtype=np.float32)
    )


def xyzi(header, points):
    return point_cloud2.create_cloud(
        header, POINT_FIELDS_XYZI, np.asarray(points, dtype=np.float32)
    )
