import geometry_msgs.msg
import numpy as np
import ros_numpy
import rospy
import tf2_ros


class TFmanager:
    buffer = None
    listener = None
    br = None
    br_static = None

    timeout: float = 0.1

    @classmethod
    def init(cls):
        assert not cls.buffer
        cls.buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(cls.buffer)
        cls.br = tf2_ros.TransformBroadcaster()
        cls.br_static = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(1)

    def __class_getitem__(cls, item) -> np.ndarray:
        """
        Usage: TFmanager[src, dst] to get the transformation matrix from `src` to `dst`.
        """
        if not cls.listener: cls.init()
        src, dst = item
        if src == dst: return np.eye(4)

        try:
            transform = cls.buffer.lookup_transform(dst, src, time=rospy.Time(0), timeout=rospy.Duration(cls.timeout))
            return ros_numpy.numpify(transform.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

    @classmethod
    def register(cls,
                 T_src_dst: np.ndarray,
                 src: str,
                 dst: str,
                 static: bool = True):
        """
        :param T_src_dst: 4x4 transformation matrix from `src` to `dst`.
        :param src: Name of the source frame.
        :param dst: Name of the destination (parent) frame.
        """
        if not cls.listener: cls.init()
        assert T_src_dst.shape == (4, 4), "Transformation matrix must be 4x4."

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = dst
        t.child_frame_id = src

        t.transform = ros_numpy.msgify(geometry_msgs.msg.Transform, T_src_dst)
        (cls.br_static if static else cls.br).sendTransform(t)


if __name__ == '__main__':
    # rosrun tf2_ros static_transform_publisher 0.1 0.2 0.3 0 0 0.785 base_link world_link
    rospy.init_node('tf_reader')

    frame = "camera_link", "panda_hand"

    """T = np.array([0.014003, -0.021309, 0.999675, 0.032479,
                  -0.000043, -0.999773, -0.02131, -0.019927,
                  0.999902, 0.000255, -0.014001, 0.047089,
                  0.0, 0.0, 0.0, 1.0]).reshape(4, 4)
    TFmanager.register(T, *frame)"""

    print(TFmanager[frame].flatten().tolist())
