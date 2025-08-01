;; Auto-generated. Do not edit!


(when (boundp 'moveit_msgs::PlaceGoal)
  (if (not (find-package "MOVEIT_MSGS"))
    (make-package "MOVEIT_MSGS"))
  (shadow 'PlaceGoal (find-package "MOVEIT_MSGS")))
(unless (find-package "MOVEIT_MSGS::PLACEGOAL")
  (make-package "MOVEIT_MSGS::PLACEGOAL"))

(in-package "ROS")
;;//! \htmlinclude PlaceGoal.msg.html


(defclass moveit_msgs::PlaceGoal
  :super ros::object
  :slots (_group_name _attached_object_name _place_locations _place_eef _support_surface_name _allow_gripper_support_collision _path_constraints _planner_id _allowed_touch_objects _allowed_planning_time _planning_options ))

(defmethod moveit_msgs::PlaceGoal
  (:init
   (&key
    ((:group_name __group_name) "")
    ((:attached_object_name __attached_object_name) "")
    ((:place_locations __place_locations) ())
    ((:place_eef __place_eef) nil)
    ((:support_surface_name __support_surface_name) "")
    ((:allow_gripper_support_collision __allow_gripper_support_collision) nil)
    ((:path_constraints __path_constraints) (instance moveit_msgs::Constraints :init))
    ((:planner_id __planner_id) "")
    ((:allowed_touch_objects __allowed_touch_objects) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:allowed_planning_time __allowed_planning_time) 0.0)
    ((:planning_options __planning_options) (instance moveit_msgs::PlanningOptions :init))
    )
   (send-super :init)
   (setq _group_name (string __group_name))
   (setq _attached_object_name (string __attached_object_name))
   (setq _place_locations __place_locations)
   (setq _place_eef __place_eef)
   (setq _support_surface_name (string __support_surface_name))
   (setq _allow_gripper_support_collision __allow_gripper_support_collision)
   (setq _path_constraints __path_constraints)
   (setq _planner_id (string __planner_id))
   (setq _allowed_touch_objects __allowed_touch_objects)
   (setq _allowed_planning_time (float __allowed_planning_time))
   (setq _planning_options __planning_options)
   self)
  (:group_name
   (&optional __group_name)
   (if __group_name (setq _group_name __group_name)) _group_name)
  (:attached_object_name
   (&optional __attached_object_name)
   (if __attached_object_name (setq _attached_object_name __attached_object_name)) _attached_object_name)
  (:place_locations
   (&rest __place_locations)
   (if (keywordp (car __place_locations))
       (send* _place_locations __place_locations)
     (progn
       (if __place_locations (setq _place_locations (car __place_locations)))
       _place_locations)))
  (:place_eef
   (&optional (__place_eef :null))
   (if (not (eq __place_eef :null)) (setq _place_eef __place_eef)) _place_eef)
  (:support_surface_name
   (&optional __support_surface_name)
   (if __support_surface_name (setq _support_surface_name __support_surface_name)) _support_surface_name)
  (:allow_gripper_support_collision
   (&optional (__allow_gripper_support_collision :null))
   (if (not (eq __allow_gripper_support_collision :null)) (setq _allow_gripper_support_collision __allow_gripper_support_collision)) _allow_gripper_support_collision)
  (:path_constraints
   (&rest __path_constraints)
   (if (keywordp (car __path_constraints))
       (send* _path_constraints __path_constraints)
     (progn
       (if __path_constraints (setq _path_constraints (car __path_constraints)))
       _path_constraints)))
  (:planner_id
   (&optional __planner_id)
   (if __planner_id (setq _planner_id __planner_id)) _planner_id)
  (:allowed_touch_objects
   (&optional __allowed_touch_objects)
   (if __allowed_touch_objects (setq _allowed_touch_objects __allowed_touch_objects)) _allowed_touch_objects)
  (:allowed_planning_time
   (&optional __allowed_planning_time)
   (if __allowed_planning_time (setq _allowed_planning_time __allowed_planning_time)) _allowed_planning_time)
  (:planning_options
   (&rest __planning_options)
   (if (keywordp (car __planning_options))
       (send* _planning_options __planning_options)
     (progn
       (if __planning_options (setq _planning_options (car __planning_options)))
       _planning_options)))
  (:serialization-length
   ()
   (+
    ;; string _group_name
    4 (length _group_name)
    ;; string _attached_object_name
    4 (length _attached_object_name)
    ;; moveit_msgs/PlaceLocation[] _place_locations
    (apply #'+ (send-all _place_locations :serialization-length)) 4
    ;; bool _place_eef
    1
    ;; string _support_surface_name
    4 (length _support_surface_name)
    ;; bool _allow_gripper_support_collision
    1
    ;; moveit_msgs/Constraints _path_constraints
    (send _path_constraints :serialization-length)
    ;; string _planner_id
    4 (length _planner_id)
    ;; string[] _allowed_touch_objects
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _allowed_touch_objects)) 4
    ;; float64 _allowed_planning_time
    8
    ;; moveit_msgs/PlanningOptions _planning_options
    (send _planning_options :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _group_name
       (write-long (length _group_name) s) (princ _group_name s)
     ;; string _attached_object_name
       (write-long (length _attached_object_name) s) (princ _attached_object_name s)
     ;; moveit_msgs/PlaceLocation[] _place_locations
     (write-long (length _place_locations) s)
     (dolist (elem _place_locations)
       (send elem :serialize s)
       )
     ;; bool _place_eef
       (if _place_eef (write-byte -1 s) (write-byte 0 s))
     ;; string _support_surface_name
       (write-long (length _support_surface_name) s) (princ _support_surface_name s)
     ;; bool _allow_gripper_support_collision
       (if _allow_gripper_support_collision (write-byte -1 s) (write-byte 0 s))
     ;; moveit_msgs/Constraints _path_constraints
       (send _path_constraints :serialize s)
     ;; string _planner_id
       (write-long (length _planner_id) s) (princ _planner_id s)
     ;; string[] _allowed_touch_objects
     (write-long (length _allowed_touch_objects) s)
     (dolist (elem _allowed_touch_objects)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64 _allowed_planning_time
       (sys::poke _allowed_planning_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; moveit_msgs/PlanningOptions _planning_options
       (send _planning_options :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _group_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _group_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _attached_object_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _attached_object_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; moveit_msgs/PlaceLocation[] _place_locations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _place_locations (let (r) (dotimes (i n) (push (instance moveit_msgs::PlaceLocation :init) r)) r))
     (dolist (elem- _place_locations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _place_eef
     (setq _place_eef (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _support_surface_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _support_surface_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _allow_gripper_support_collision
     (setq _allow_gripper_support_collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; moveit_msgs/Constraints _path_constraints
     (send _path_constraints :deserialize buf ptr-) (incf ptr- (send _path_constraints :serialization-length))
   ;; string _planner_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _planner_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _allowed_touch_objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _allowed_touch_objects (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _allowed_touch_objects i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64 _allowed_planning_time
     (setq _allowed_planning_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; moveit_msgs/PlanningOptions _planning_options
     (send _planning_options :deserialize buf ptr-) (incf ptr- (send _planning_options :serialization-length))
   ;;
   self)
  )

(setf (get moveit_msgs::PlaceGoal :md5sum-) "b5ff24625cebec440c18cd2e1d833764")
(setf (get moveit_msgs::PlaceGoal :datatype-) "moveit_msgs/PlaceGoal")
(setf (get moveit_msgs::PlaceGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# An action for placing an object

# which group to be used to plan for grasping
string group_name

# the name of the attached object to place
string attached_object_name

# a list of possible transformations for placing the object
PlaceLocation[] place_locations

# if the user prefers setting the eef pose (same as in pick) rather than
# the location of the object, this flag should be set to true
bool place_eef

# the name that the support surface (e.g. table) has in the collision world
# can be left empty if no name is available
string support_surface_name

# whether collisions between the gripper and the support surface should be acceptable
# during move from pre-place to place and during retreat. Collisions when moving to the
# pre-place location are still not allowed even if this is set to true.
bool allow_gripper_support_collision

# Optional constraints to be imposed on every point in the motion plan
Constraints path_constraints

# The name of the motion planner to use. If no name is specified,
# a default motion planner will be used
string planner_id

# an optional list of obstacles that we have semantic information about
# and that can be touched/pushed/moved in the course of placing
string[] allowed_touch_objects

# The maximum amount of time the motion planner is allowed to plan for
float64 allowed_planning_time

# Planning options
PlanningOptions planning_options


================================================================================
MSG: moveit_msgs/PlaceLocation
# A name for this grasp
string id

# The internal posture of the hand for the grasp
# positions and efforts are used
trajectory_msgs/JointTrajectory post_place_posture

# The position of the end-effector for the grasp relative to a reference frame
# (that is always specified elsewhere, not in this message)
geometry_msgs/PoseStamped place_pose

# The estimated probability of success for this place, or some other
# measure of how \"good\" it is.
float64 quality

# The approach motion
GripperTranslation pre_place_approach

# The retreat motion
GripperTranslation post_place_retreat

# an optional list of obstacles that we have semantic information about
# and that can be touched/pushed/moved in the course of grasping
string[] allowed_touch_objects

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: moveit_msgs/GripperTranslation
# defines a translation for the gripper, used in pickup or place tasks
# for example for lifting an object off a table or approaching the table for placing

# the direction of the translation
geometry_msgs/Vector3Stamped direction

# the desired translation distance
float32 desired_distance

# the min distance that must be considered feasible before the
# grasp is even attempted
float32 min_distance

================================================================================
MSG: geometry_msgs/Vector3Stamped
# This represents a Vector3 with reference coordinate frame and timestamp
Header header
Vector3 vector

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
MSG: moveit_msgs/Constraints
# This message contains a list of motion planning constraints.
# All constraints must be satisfied for a goal to be considered valid

string name

JointConstraint[] joint_constraints

PositionConstraint[] position_constraints

OrientationConstraint[] orientation_constraints

VisibilityConstraint[] visibility_constraints

================================================================================
MSG: moveit_msgs/JointConstraint
# Constrain the position of a joint to be within a certain bound
string joint_name

# the bound to be achieved is [position - tolerance_below, position + tolerance_above]
float64 position
float64 tolerance_above
float64 tolerance_below

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight

================================================================================
MSG: moveit_msgs/PositionConstraint
# This message contains the definition of a position constraint.

Header header

# The robot link this constraint refers to
string link_name

# The offset (in the link frame) for the target point on the link we are planning for
geometry_msgs/Vector3 target_point_offset

# The volume this constraint refers to
BoundingVolume constraint_region

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight

================================================================================
MSG: moveit_msgs/BoundingVolume
# Define a volume in 3D

# A set of solid geometric primitives that make up the volume to define (as a union)
shape_msgs/SolidPrimitive[] primitives

# The poses at which the primitives are located
geometry_msgs/Pose[] primitive_poses

# In addition to primitives, meshes can be specified to add to the bounding volume (again, as union)
shape_msgs/Mesh[] meshes

# The poses at which the meshes are located
geometry_msgs/Pose[] mesh_poses

================================================================================
MSG: shape_msgs/SolidPrimitive
# Define box, sphere, cylinder, cone 
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type


# The dimensions of the shape
float64[] dimensions

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array

# For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
# sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2


# For the SPHERE type, only one component is used, and it gives the radius of
# the sphere.
uint8 SPHERE_RADIUS=0


# For the CYLINDER and CONE types, the center line is oriented along
# the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
# of dimensions gives the height of the cylinder (cone).  The
# CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
# radius of the base of the cylinder (cone).  Cone and cylinder
# primitives are defined to be circular. The tip of the cone is
# pointing up, along +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

================================================================================
MSG: shape_msgs/Mesh
# Definition of a mesh

# list of triangles; the index values refer to positions in vertices[]
MeshTriangle[] triangles

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices

================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices
uint32[3] vertex_indices

================================================================================
MSG: moveit_msgs/OrientationConstraint
# This message contains the definition of an orientation constraint.

Header header

# The desired orientation of the robot link specified as a quaternion
geometry_msgs/Quaternion orientation

# The robot link this constraint refers to
string link_name

# Tolerance on the three vector components of the orientation error (optional)
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance

# Defines how the orientation error is calculated
# The error is compared to the tolerance defined above
uint8 parameterization

# The different options for the orientation error parameterization
# - Intrinsic xyz Euler angles (default value)
uint8 XYZ_EULER_ANGLES=0
# - A rotation vector. This is similar to the angle-axis representation,
# but the magnitude of the vector represents the rotation angle.
uint8 ROTATION_VECTOR=1

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight

================================================================================
MSG: moveit_msgs/VisibilityConstraint
# The constraint is useful to maintain visibility to a disc (the target) in a particular frame.
# This disc forms the base of a visibiliy cone whose tip is at the origin of the sensor.
# Maintaining visibility is done by ensuring the robot does not obstruct the visibility cone.
# Note:
# This constraint does NOT enforce minimum or maximum distances between the sensor
# and the target, nor does it enforce the target to be in the field of view of
# the sensor. A PositionConstraint can (and probably should) be used for such purposes.

# The radius of the disc that should be maintained visible
float64 target_radius

# The pose of the disc; as the robot moves, the pose of the disc may change as well
# This can be in the frame of a particular robot link, for example
geometry_msgs/PoseStamped target_pose

# From the sensor origin towards the target, the disc forms a visibility cone
# This cone is approximated using many sides. For example, when using 4 sides,
# that in fact makes the visibility region be a pyramid.
# This value should always be 3 or more.
int32 cone_sides

# The pose in which visibility is to be maintained.
# The frame id should represent the robot link to which the sensor is attached.
# It is assumed the sensor can look directly at the target, in any direction.
# This assumption is usually not true, but additional PositionConstraints
# can resolve this issue.
geometry_msgs/PoseStamped sensor_pose

# Even though the disc is maintained visible, the visibility cone can be very small
# because of the orientation of the disc with respect to the sensor. It is possible
# for example to view the disk almost from a side, in which case the visibility cone
# can end up having close to 0 volume. The view angle is defined to be the angle between
# the normal to the visibility disc and the direction vector from the sensor origin.
# The value below represents the minimum desired view angle. For a perfect view,
# this value will be 0 (the two vectors are exact opposites). For a completely obstructed view
# this value will be Pi/2 (the vectors are perpendicular). This value defined below
# is the maximum view angle to be maintained. This should be a value in the open interval
# (0, Pi/2). If 0 is set, the view angle is NOT enforced.
float64 max_view_angle

# This angle is used similarly to max_view_angle but limits the maximum angle
# between the sensor origin direction vector and the axis that connects the
# sensor origin to the target frame origin. The value is again in the range (0, Pi/2)
# and is NOT enforced if set to 0.
float64 max_range_angle

# The axis that is assumed to indicate the direction of view for the sensor
# X = 2, Y = 1, Z = 0
uint8 SENSOR_Z=0
uint8 SENSOR_Y=1
uint8 SENSOR_X=2
uint8 sensor_view_direction

# A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
float64 weight

================================================================================
MSG: moveit_msgs/PlanningOptions

# The diff to consider for the planning scene (optional)
PlanningScene planning_scene_diff

# If this flag is set to true, the action
# returns an executable plan in the response but does not attempt execution
bool plan_only

# If this flag is set to true, the action of planning &
# executing is allowed to look around  (move sensors) if
# it seems that not enough information is available about
# the environment
bool look_around

# If this value is positive, the action of planning & executing
# is allowed to look around for a maximum number of attempts;
# If the value is left as 0, the default value is used, as set
# with dynamic_reconfigure
int32 look_around_attempts

# If set and if look_around is true, this value is used as
# the maximum cost allowed for a path to be considered executable.
# If the cost of a path is higher than this value, more sensing or
# a new plan needed. If left as 0.0 but look_around is true, then
# the default value set via dynamic_reconfigure is used
float64 max_safe_execution_cost

# If the plan becomes invalidated during execution, it is possible to have
# that plan recomputed and execution restarted. This flag enables this
# functionality
bool replan

# The maximum number of replanning attempts
int32 replan_attempts

# The amount of time to wait in between replanning attempts (in seconds)
float64 replan_delay

================================================================================
MSG: moveit_msgs/PlanningScene
# name of planning scene
string name

# full robot state
RobotState robot_state

# The name of the robot model this scene is for
string robot_model_name

#additional frames for duplicating tf (with respect to the planning frame)
geometry_msgs/TransformStamped[] fixed_frame_transforms

#full allowed collision matrix
AllowedCollisionMatrix allowed_collision_matrix

# all link paddings
LinkPadding[] link_padding

# all link scales
LinkScale[] link_scale

# Attached objects, collision objects, even the octomap or collision map can have
# colors associated to them. This array specifies them.
ObjectColor[] object_colors

# the collision map
PlanningSceneWorld world

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
bool is_diff

================================================================================
MSG: moveit_msgs/RobotState
# This message contains information about the robot state, i.e. the positions of its joints and links
sensor_msgs/JointState joint_state

# Joints that may have multiple DOF are specified here
sensor_msgs/MultiDOFJointState multi_dof_joint_state

# Attached collision objects (attached to some link on the robot)
AttachedCollisionObject[] attached_collision_objects

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
# This is mostly important for handling the attached bodies (whether or not to clear the attached bodies
# of a moveit::core::RobotState before updating it with this message)
bool is_diff

================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: sensor_msgs/MultiDOFJointState
# Representation of state for joints with multiple degrees of freedom, 
# following the structure of JointState.
#
# It is assumed that a joint in a system corresponds to a transform that gets applied 
# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)
# and those 3DOF can be expressed as a transformation matrix, and that transformation
# matrix can be converted back to (x, y, yaw)
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# wrench associated with them, you can leave the wrench array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

Header header

string[] joint_names
geometry_msgs/Transform[] transforms
geometry_msgs/Twist[] twist
geometry_msgs/Wrench[] wrench

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: moveit_msgs/AttachedCollisionObject
# The CollisionObject will be attached with a fixed joint to this link
string link_name

#This contains the actual shapes and poses for the CollisionObject
#to be attached to the link
#If action is remove and no object.id is set, all objects
#attached to the link indicated by link_name will be removed
CollisionObject object

# The set of links that the attached objects are allowed to touch
# by default - the link_name is already considered by default
string[] touch_links

# If certain links were placed in a particular posture for this object to remain attached
# (e.g., an end effector closing around an object), the posture necessary for releasing
# the object is stored here
trajectory_msgs/JointTrajectory detach_posture

# The weight of the attached object, if known
float64 weight

================================================================================
MSG: moveit_msgs/CollisionObject
# A header, used for interpreting the poses
Header header

# The object's pose relative to the header frame.
# The shapes and subframe poses are defined relative to this pose.
geometry_msgs/Pose pose

# The id of the object (name used in MoveIt)
string id

# The object type in a database of known objects
object_recognition_msgs/ObjectType type

# The collision geometries associated with the object.
# Their poses are with respect to the object's pose

# Solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

# Meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses

# Named subframes on the object. Use these to define points of interest on the object that you want
# to plan with (e.g. \"tip\", \"spout\", \"handle\"). The id of the object will be prepended to the subframe.
# If an object with the id \"screwdriver\" and a subframe \"tip\" is in the scene, you can use the frame
# \"screwdriver/tip\" for planning.
# The length of the subframe_names and subframe_poses has to be identical.
string[] subframe_names
geometry_msgs/Pose[] subframe_poses

# Adds the object to the planning scene. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# Append to an object that already exists in the planning scene. If the object does not exist, it is added.
byte APPEND=2

# If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
# if solely moving the object is desired
byte MOVE=3

# Operation to be performed
byte operation

================================================================================
MSG: object_recognition_msgs/ObjectType
################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db

================================================================================
MSG: shape_msgs/Plane
# Representation of a plane, using the plane equation ax + by + cz + d = 0

# a := coef[0]
# b := coef[1]
# c := coef[2]
# d := coef[3]

float64[4] coef

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: moveit_msgs/AllowedCollisionMatrix
# The list of entry names in the matrix
string[] entry_names

# The individual entries in the allowed collision matrix
# square, symmetric, with same order as entry_names
AllowedCollisionEntry[] entry_values

# In addition to the collision matrix itself, we also have
# the default entry value for each entry name.

# If the allowed collision flag is queried for a pair of names (n1, n2)
# that is not found in the collision matrix itself, the value of
# the collision flag is considered to be that of the entry (n1 or n2)
# specified in the list below. If both n1 and n2 are found in the list
# of defaults, the result is computed with an AND operation

string[] default_entry_names
bool[] default_entry_values

================================================================================
MSG: moveit_msgs/AllowedCollisionEntry
# whether or not collision checking is enabled
bool[] enabled

================================================================================
MSG: moveit_msgs/LinkPadding
#name for the link
string link_name

# padding to apply to the link
float64 padding

================================================================================
MSG: moveit_msgs/LinkScale
#name for the link
string link_name

# scaling to apply to the link
float64 scale

================================================================================
MSG: moveit_msgs/ObjectColor
# The object id for which we specify color
string id

# The value of the color
std_msgs/ColorRGBA color

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

================================================================================
MSG: moveit_msgs/PlanningSceneWorld
# collision objects
CollisionObject[] collision_objects

# The octomap that represents additional collision data
octomap_msgs/OctomapWithPose octomap

================================================================================
MSG: octomap_msgs/OctomapWithPose
# A 3D map in binary format, as Octree
Header header

# The pose of the octree with respect to the header frame 
geometry_msgs/Pose origin

# The actual octree msg
octomap_msgs/Octomap octomap

================================================================================
MSG: octomap_msgs/Octomap
# A 3D map in binary format, as Octree
Header header

# Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
bool binary

# Class id of the contained octree 
string id

# Resolution (in m) of the smallest octree nodes
float64 resolution

# binary serialization of octree, use conversions.h to read and write octrees
int8[] data

")



(provide :moveit_msgs/PlaceGoal "b5ff24625cebec440c18cd2e1d833764")


