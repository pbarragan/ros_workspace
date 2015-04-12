; Auto-generated. Do not edit!


(cl:in-package ee_cart_imped_control-msg)


;//! \htmlinclude EECartImpedActionGoal.msg.html

(cl:defclass <EECartImpedActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type ee_cart_imped_control-msg:EECartImpedGoal
    :initform (cl:make-instance 'ee_cart_imped_control-msg:EECartImpedGoal)))
)

(cl:defclass EECartImpedActionGoal (<EECartImpedActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EECartImpedActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EECartImpedActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee_cart_imped_control-msg:<EECartImpedActionGoal> is deprecated: use ee_cart_imped_control-msg:EECartImpedActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EECartImpedActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_control-msg:header-val is deprecated.  Use ee_cart_imped_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <EECartImpedActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_control-msg:goal_id-val is deprecated.  Use ee_cart_imped_control-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <EECartImpedActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_control-msg:goal-val is deprecated.  Use ee_cart_imped_control-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EECartImpedActionGoal>) ostream)
  "Serializes a message object of type '<EECartImpedActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EECartImpedActionGoal>) istream)
  "Deserializes a message object of type '<EECartImpedActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EECartImpedActionGoal>)))
  "Returns string type for a message object of type '<EECartImpedActionGoal>"
  "ee_cart_imped_control/EECartImpedActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECartImpedActionGoal)))
  "Returns string type for a message object of type 'EECartImpedActionGoal"
  "ee_cart_imped_control/EECartImpedActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EECartImpedActionGoal>)))
  "Returns md5sum for a message object of type '<EECartImpedActionGoal>"
  "09251e687b897c24590030b27467eb9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EECartImpedActionGoal)))
  "Returns md5sum for a message object of type 'EECartImpedActionGoal"
  "09251e687b897c24590030b27467eb9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EECartImpedActionGoal>)))
  "Returns full string definition for message of type '<EECartImpedActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%EECartImpedGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ee_cart_imped_control/EECartImpedGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%Header header~%ee_cart_imped_control/StiffPoint[] trajectory~%~%================================================================================~%MSG: ee_cart_imped_control/StiffPoint~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Wrench wrench_or_stiffness~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EECartImpedActionGoal)))
  "Returns full string definition for message of type 'EECartImpedActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%EECartImpedGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ee_cart_imped_control/EECartImpedGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%Header header~%ee_cart_imped_control/StiffPoint[] trajectory~%~%================================================================================~%MSG: ee_cart_imped_control/StiffPoint~%Header header~%geometry_msgs/Pose pose~%geometry_msgs/Wrench wrench_or_stiffness~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EECartImpedActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EECartImpedActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'EECartImpedActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
