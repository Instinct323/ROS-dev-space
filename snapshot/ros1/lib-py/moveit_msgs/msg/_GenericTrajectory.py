# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from moveit_msgs/GenericTrajectory.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg
import moveit_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg

class GenericTrajectory(genpy.Message):
  _md5sum = "d68b5c73072efa2012238a77e49c2c58"
  _type = "moveit_msgs/GenericTrajectory"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# A generic trajectory message that may either encode a joint- or cartesian-space trajectory, or both
# Trajectories encoded in this message are considered semantically equivalent
Header header
trajectory_msgs/JointTrajectory[] joint_trajectory
moveit_msgs/CartesianTrajectory[] cartesian_trajectory

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start

================================================================================
MSG: moveit_msgs/CartesianTrajectory
# This message describes the trajectory of a tracked frame in task-space
Header header

# The name of the Cartesian frame being tracked with respect to the base frame provided in header.frame_id
string tracked_frame

CartesianTrajectoryPoint[] points

================================================================================
MSG: moveit_msgs/CartesianTrajectoryPoint
# The definition of a cartesian point in a trajectory. Defines the cartesian state of the point and it's time,
# following the pattern of the JointTrajectory message
CartesianPoint point

duration time_from_start

================================================================================
MSG: moveit_msgs/CartesianPoint
# This message defines a point in a cartesian trajectory
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
"""
  __slots__ = ['header','joint_trajectory','cartesian_trajectory']
  _slot_types = ['std_msgs/Header','trajectory_msgs/JointTrajectory[]','moveit_msgs/CartesianTrajectory[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,joint_trajectory,cartesian_trajectory

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GenericTrajectory, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_trajectory is None:
        self.joint_trajectory = []
      if self.cartesian_trajectory is None:
        self.cartesian_trajectory = []
    else:
      self.header = std_msgs.msg.Header()
      self.joint_trajectory = []
      self.cartesian_trajectory = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.joint_trajectory)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_trajectory:
        _v1 = val1.header
        _x = _v1.seq
        buff.write(_get_struct_I().pack(_x))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(val1.joint_names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.joint_names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.Struct('<I%ss'%length).pack(length, val2))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.positions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.Struct(pattern).pack(*val2.positions))
          length = len(val2.velocities)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.Struct(pattern).pack(*val2.velocities))
          length = len(val2.accelerations)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.Struct(pattern).pack(*val2.accelerations))
          length = len(val2.effort)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.Struct(pattern).pack(*val2.effort))
          _v3 = val2.time_from_start
          _x = _v3
          buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
      length = len(self.cartesian_trajectory)
      buff.write(_struct_I.pack(length))
      for val1 in self.cartesian_trajectory:
        _v4 = val1.header
        _x = _v4.seq
        buff.write(_get_struct_I().pack(_x))
        _v5 = _v4.stamp
        _x = _v5
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v4.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.tracked_frame
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _v6 = val2.point
          _v7 = _v6.pose
          _v8 = _v7.position
          _x = _v8
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v9 = _v7.orientation
          _x = _v9
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          _v10 = _v6.velocity
          _v11 = _v10.linear
          _x = _v11
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v12 = _v10.angular
          _x = _v12
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v13 = _v6.acceleration
          _v14 = _v13.linear
          _x = _v14
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v15 = _v13.angular
          _x = _v15
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v16 = val2.time_from_start
          _x = _v16
          buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_trajectory is None:
        self.joint_trajectory = None
      if self.cartesian_trajectory is None:
        self.cartesian_trajectory = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_trajectory = []
      for i in range(0, length):
        val1 = trajectory_msgs.msg.JointTrajectory()
        _v17 = val1.header
        start = end
        end += 4
        (_v17.seq,) = _get_struct_I().unpack(str[start:end])
        _v18 = _v17.stamp
        _x = _v18
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v17.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v17.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.joint_names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2 = str[start:end]
          val1.joint_names.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = trajectory_msgs.msg.JointTrajectoryPoint()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.positions = s.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.velocities = s.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.accelerations = s.unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.effort = s.unpack(str[start:end])
          _v19 = val2.time_from_start
          _x = _v19
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
          val1.points.append(val2)
        self.joint_trajectory.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cartesian_trajectory = []
      for i in range(0, length):
        val1 = moveit_msgs.msg.CartesianTrajectory()
        _v20 = val1.header
        start = end
        end += 4
        (_v20.seq,) = _get_struct_I().unpack(str[start:end])
        _v21 = _v20.stamp
        _x = _v21
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v20.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v20.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.tracked_frame = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.tracked_frame = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = moveit_msgs.msg.CartesianTrajectoryPoint()
          _v22 = val2.point
          _v23 = _v22.pose
          _v24 = _v23.position
          _x = _v24
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v25 = _v23.orientation
          _x = _v25
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          _v26 = _v22.velocity
          _v27 = _v26.linear
          _x = _v27
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v28 = _v26.angular
          _x = _v28
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v29 = _v22.acceleration
          _v30 = _v29.linear
          _x = _v30
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v31 = _v29.angular
          _x = _v31
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v32 = val2.time_from_start
          _x = _v32
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
          val1.points.append(val2)
        self.cartesian_trajectory.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.joint_trajectory)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_trajectory:
        _v33 = val1.header
        _x = _v33.seq
        buff.write(_get_struct_I().pack(_x))
        _v34 = _v33.stamp
        _x = _v34
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v33.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(val1.joint_names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.joint_names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.Struct('<I%ss'%length).pack(length, val2))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.positions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.positions.tostring())
          length = len(val2.velocities)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.velocities.tostring())
          length = len(val2.accelerations)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.accelerations.tostring())
          length = len(val2.effort)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.effort.tostring())
          _v35 = val2.time_from_start
          _x = _v35
          buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
      length = len(self.cartesian_trajectory)
      buff.write(_struct_I.pack(length))
      for val1 in self.cartesian_trajectory:
        _v36 = val1.header
        _x = _v36.seq
        buff.write(_get_struct_I().pack(_x))
        _v37 = _v36.stamp
        _x = _v37
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v36.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.tracked_frame
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          _v38 = val2.point
          _v39 = _v38.pose
          _v40 = _v39.position
          _x = _v40
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v41 = _v39.orientation
          _x = _v41
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          _v42 = _v38.velocity
          _v43 = _v42.linear
          _x = _v43
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v44 = _v42.angular
          _x = _v44
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v45 = _v38.acceleration
          _v46 = _v45.linear
          _x = _v46
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v47 = _v45.angular
          _x = _v47
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v48 = val2.time_from_start
          _x = _v48
          buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_trajectory is None:
        self.joint_trajectory = None
      if self.cartesian_trajectory is None:
        self.cartesian_trajectory = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_trajectory = []
      for i in range(0, length):
        val1 = trajectory_msgs.msg.JointTrajectory()
        _v49 = val1.header
        start = end
        end += 4
        (_v49.seq,) = _get_struct_I().unpack(str[start:end])
        _v50 = _v49.stamp
        _x = _v50
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v49.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v49.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.joint_names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2 = str[start:end]
          val1.joint_names.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = trajectory_msgs.msg.JointTrajectoryPoint()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.positions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.velocities = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.accelerations = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          s = struct.Struct(pattern)
          end += s.size
          val2.effort = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          _v51 = val2.time_from_start
          _x = _v51
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
          val1.points.append(val2)
        self.joint_trajectory.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cartesian_trajectory = []
      for i in range(0, length):
        val1 = moveit_msgs.msg.CartesianTrajectory()
        _v52 = val1.header
        start = end
        end += 4
        (_v52.seq,) = _get_struct_I().unpack(str[start:end])
        _v53 = _v52.stamp
        _x = _v53
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v52.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v52.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.tracked_frame = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.tracked_frame = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = moveit_msgs.msg.CartesianTrajectoryPoint()
          _v54 = val2.point
          _v55 = _v54.pose
          _v56 = _v55.position
          _x = _v56
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v57 = _v55.orientation
          _x = _v57
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          _v58 = _v54.velocity
          _v59 = _v58.linear
          _x = _v59
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v60 = _v58.angular
          _x = _v60
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v61 = _v54.acceleration
          _v62 = _v61.linear
          _x = _v62
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v63 = _v61.angular
          _x = _v63
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v64 = val2.time_from_start
          _x = _v64
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
          val1.points.append(val2)
        self.cartesian_trajectory.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
