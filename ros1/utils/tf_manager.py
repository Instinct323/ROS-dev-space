import threading

import geometry_msgs.msg
import numpy as np
import ros_numpy
import rospy
import tf2_ros

TFgraph = dict[str, dict[str, geometry_msgs.msg.TransformStamped]]


def parse_transform(value: list[float] | np.ndarray) -> np.ndarray:
    return np.array(value, dtype=float).reshape(4, 4)


class TFmanager:
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.))
    br = tf2_ros.TransformBroadcaster()
    br_static = tf2_ros.StaticTransformBroadcaster()

    ns: str = ""
    remap: dict[str, str] = {}

    listener: tf2_ros.TransformListener = None
    tf: TFgraph = {}
    tf_static: TFgraph = {}

    @classmethod
    def init(cls,
             pause: float = 1.):
        if cls.listener: return
        cls.listener = tf2_ros.TransformListener(cls.buffer)
        rospy.sleep(pause)

    @classmethod
    def load_yaml(cls,
                  file: str,
                  broadcast: bool = False):
        """
        Load transformations from a YAML file and register them.
        """
        import yaml
        with open(file, 'r') as f:
            cfg = yaml.load(f.read(), Loader=yaml.Loader)
            cls.ns = cfg["namespace"]
            cls.remap = cfg["remap"]

            # load static transforms
            for dst, src_T in cfg["transform"].items():
                for src, T in src_T.items():
                    cls.register(parse_transform(T), src, dst, static=True, broadcast=broadcast)

    def __class_getitem__(cls, item) -> np.ndarray:
        """
        Usage: TFmanager[src, dst] to get the transformation matrix from `src` to `dst`.
        """
        msg = cls.query(*item)
        if msg: return ros_numpy.numpify(msg.transform)

    @classmethod
    def local_frame(cls,
                    frame: str) -> str:
        """
        Get the local frame name with namespace and remapping applied.
        """
        try:
            frame = cls.remap[frame]
        except KeyError:
            frame = cls.ns + frame
        return frame

    @classmethod
    def query(cls,
              src: str,
              dst: str,
              timestamp: float | rospy.Time = 0.,
              timeout: float | rospy.Duration = 0.5,
              locally: bool = True) -> geometry_msgs.msg.TransformStamped:
        """
        Query the transformation from `src` to `dst` at a specific timestamp.
        """
        cls.init()
        if src == dst: return np.eye(4)
        if locally: src, dst = map(cls.local_frame, (src, dst))

        # query
        if not isinstance(timestamp, rospy.Time): timestamp = rospy.Time(timestamp)
        if not isinstance(timeout, rospy.Duration): timeout = rospy.Duration(timeout)
        try:
            return cls.buffer.lookup_transform(dst, src, time=timestamp, timeout=timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

    @classmethod
    def register(cls,
                 T_src_dst: np.ndarray,
                 src: str,
                 dst: str,
                 timestamp: rospy.Time = None,
                 static: bool = False,
                 broadcast: bool = False,
                 locally: bool = True):
        """
        :param T_src_dst: 4x4 transformation matrix from `src` to `dst`
        :param src: source (child) frame
        :param dst: destination (parent) frame
        :param timestamp: time of the transform
        :param static: whether the transform is static (does not change over time)
        :param broadcast: whether to broadcast the transform
        :param locally: whether to add the namespace to the frame names
        """
        cls.init()
        assert T_src_dst.shape == (4, 4), "Transformation matrix must be 4x4."
        if locally: src, dst = map(cls.local_frame, (src, dst))

        # generate message
        t = geometry_msgs.msg.TransformStamped()
        t.child_frame_id = src
        t.header.frame_id = dst
        t.header.stamp = timestamp or rospy.Time.now()
        t.transform = ros_numpy.msgify(geometry_msgs.msg.Transform, T_src_dst)

        # update TFgraph
        graph = cls.tf_static if static else cls.tf
        graph.setdefault(dst, {})[src] = t
        assert graph.get(src, {}).get(dst) is None, f"Conflict transform between {src} and {dst}."
        msg_list = sum((list(x.values()) for x in graph.values()), [])

        # send transform
        if broadcast:
            (cls.br_static if static else cls.br).sendTransform(msg_list)
        else:
            (cls.buffer.set_transform_static if static else cls.buffer.set_transform)(msg_list, "custom")

    @classmethod
    def auto_publish(cls,
                     src: str,
                     dst: str,
                     topic: str,
                     queue_size: int,
                     timeout: float) -> threading.Event:
        """
        Automatically publish the transform from `src` to `dst` at regular intervals.
        :return: a threading.Event that can be set to stop the publishing.
        """
        timeout = rospy.Duration(timeout)
        pub = rospy.Publisher(topic, geometry_msgs.msg.TransformStamped, queue_size=queue_size)
        event = threading.Event()

        def task():
            while not (rospy.is_shutdown() or event.is_set()):
                msg = cls.query(src, dst, timeout=timeout)
                if msg: pub.publish(msg)

        threading.Thread(target=task).start()
        return event


if __name__ == '__main__':
    import sys

    sys.path.append("/media/tongzj/Data/Workbench")
    from ModelsAPI.api.utils.pose_tf import parse_transform

    # rosrun tf2_ros static_transform_publisher 0.1 0.2 0.3 0 0 0.785 base_link world_link
    rospy.init_node('tf_reader')

    TFmanager.load_yaml("/media/tongzj/Data/Workbench/Lab/ClutterTOG/config/transform.yaml", broadcast=True)
    print(TFmanager["camera", "imu"])

    TFmanager.auto_publish("camera", "map", topic="tmp_tf", queue_size=10, timeout=0.5)
    # rospy.spin()
