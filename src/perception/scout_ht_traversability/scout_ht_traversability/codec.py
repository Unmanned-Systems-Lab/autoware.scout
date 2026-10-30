"""Strict ROS geometry and GridMap decoding; never guess missing array layout."""
import math

import numpy as np

from .core import Geometry, Pose2, Terrain


def planar_pose(position, orientation):
    q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError("Invalid quaternion")
    # These maps must be gravity aligned; accepting tilt would distort footprints.
    if abs(q[0]) > 5e-4 or abs(q[1]) > 5e-4:
        raise ValueError("Nonplanar map/transform orientation")
    if not math.isfinite(position.z):
        raise ValueError("Invalid height")
    return Pose2(position.x, position.y,
                 math.atan2(2 * q[3] * q[2], 1 - 2 * q[2] * q[2]))


def costmap_geometry(message):
    info = message.info
    if not message.header.frame_id:
        raise ValueError("Costmap has no frame")
    geometry = Geometry(int(info.width), int(info.height), float(info.resolution),
                        planar_pose(info.origin.position, info.origin.orientation))
    if len(message.data) != geometry.width * geometry.height:
        raise ValueError("Costmap data length mismatch")
    return geometry


def decode_terrain(message):
    info = message.info
    if (not message.header.frame_id or not np.isfinite(
            [info.resolution, info.length_x, info.length_y]).all()
            or min(info.resolution, info.length_x, info.length_y) <= 0):
        raise ValueError("Invalid GridMap geometry")
    sizes = np.array([info.length_x, info.length_y]) / info.resolution
    shape = tuple(int(round(v)) for v in sizes)
    if (min(shape) <= 0 or np.prod(shape) > 1_000_000
            or not np.allclose(sizes, shape, atol=1e-4, rtol=0)):
        raise ValueError("GridMap lengths do not match its resolution")
    if (len(message.layers) != len(message.data)
            or len(set(message.layers)) != len(message.layers)):
        raise ValueError("Invalid GridMap layer list")
    layers = dict(zip(message.layers, message.data))
    start = (int(message.outer_start_index), int(message.inner_start_index))
    if any(v < 0 or v >= n for v, n in zip(start, shape)):
        raise ValueError("Invalid GridMap circular buffer indices")

    def layer(name):
        if name not in layers:
            raise ValueError("Missing HT layer: " + name)
        msg = layers[name]
        dims = msg.layout.dim
        if len(dims) != 2:
            raise ValueError("HT layer requires a two-dimensional layout")
        labels = tuple(d.label for d in dims)
        if labels == ("column_index", "row_index"):
            actual, order, inner = (dims[1].size, dims[0].size), "F", shape[0]
        elif labels == ("row_index", "column_index"):
            actual, order, inner = (dims[0].size, dims[1].size), "C", shape[1]
        else:
            raise ValueError("Unknown HT matrix layout")
        offset = int(msg.layout.data_offset)
        size = shape[0] * shape[1]
        if (actual != shape or dims[0].stride != size or dims[1].stride != inner
                or offset < 0 or len(msg.data) != offset + size):
            raise ValueError("Inconsistent HT layer size/stride")
        matrix = np.asarray(msg.data, dtype=np.float32)[offset:].reshape(shape, order=order)
        return np.roll(matrix, (-start[0], -start[1]), axis=(0, 1)).copy()

    names = [f"ht_dir_valid_{k}" for k in range(8)]
    if any(name in layers for name in names) and not all(name in layers for name in names):
        raise ValueError("Incomplete directional support layers")
    return Terrain(
        np.stack([layer(f"ht_dir_{k}") for k in range(8)]), layer("ht_valid"),
        float(info.resolution), planar_pose(info.pose.position, info.pose.orientation),
        np.stack([layer(name) for name in names]) if all(n in layers for n in names) else None,
        layer("ht_observed") if "ht_observed" in layers else None,
    )
