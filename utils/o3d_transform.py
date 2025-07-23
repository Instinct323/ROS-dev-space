import geometry_msgs.msg
import numpy as np
import open3d as o3d
import ros_numpy
import sensor_msgs.msg
import std_msgs.msg
import visualization_msgs.msg


def to_open3d(msg):
    # sensor_msgs.msg.PointCloud2 -> o3d.geometry.PointCloud
    if isinstance(msg, sensor_msgs.msg.PointCloud2):
        struct = ros_numpy.numpify(msg)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.stack([struct["x"], struct["y"], struct["z"]], axis=-1))

        if "rgb" in struct.dtype.names:
            rgb = struct["rgb"]
            r = (rgb >> 16) & 0xFF
            g = (rgb >> 8) & 0xFF
            b = rgb & 0xFF
            pcd.colors = o3d.utility.Vector3dVector(np.stack([r, g, b], axis=-1) / 255.0)
        return pcd

    else:
        raise TypeError(f"Unsupported message type: {type(msg)}.")


def msgify(geomerty, *args, **kwargs):
    # o3d.geometry.PointCloud -> sensor_msgs.msg.PointCloud2
    if isinstance(geomerty, o3d.geometry.PointCloud):
        dtype = [(axis, np.float32) for axis in "xyz"]
        if geomerty.has_colors(): dtype.append(("rgb", np.uint32))

        struct = np.zeros(len(geomerty.points), dtype=dtype)
        struct["x"], struct["y"], struct["z"] = np.asarray(geomerty.points).T

        if geomerty.has_colors():
            rgb = np.round(np.asarray(geomerty.colors) * 255).astype(np.uint32)
            struct["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        return ros_numpy.msgify(sensor_msgs.msg.PointCloud2, struct, *args, **kwargs)

    # o3d.geometry.TriangleMesh -> visualization_msgs.msg.Marker
    elif isinstance(geomerty, o3d.geometry.TriangleMesh):
        Marker = visualization_msgs.msg.Marker
        marker = Marker()
        marker.type = Marker.TRIANGLE_LIST
        marker.header.frame_id = kwargs.get("frame_id", "")
        marker.action = kwargs.get("action", Marker.ADD)

        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0

        vertices = np.array(list(map(lambda x: ros_numpy.msgify(geometry_msgs.msg.Point, x), np.asarray(geomerty.vertices))))
        colors = None
        if geomerty.has_vertex_colors():
            colors = np.array([std_msgs.msg.ColorRGBA(*rgb, 1.0) for rgb in np.asarray(geomerty.vertex_colors)])

        triangles = np.asarray(geomerty.triangles).flatten()
        marker.points = vertices[triangles].tolist()
        if colors is not None:
            marker.colors = colors[triangles].tolist()
        return marker

    else:
        raise TypeError(f"Unsupported geometry type: {type(geomerty)}.")


if __name__ == '__main__':
    import rospy

    # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    mesh = o3d.io.read_triangle_mesh("/media/tongzj/Data/Workbench/assets/camera.ply")

    rospy.init_node("test")
    pub = rospy.Publisher('/visualization_marker', visualization_msgs.msg.Marker, queue_size=10)

    while not rospy.is_shutdown():
        msg = msgify(mesh, frame_id="map")
        pub.publish(msg)
        rospy.sleep(rospy.Duration(0.01))
