; Auto-generated. Do not edit!


(cl:in-package ee_force-msg)


;//! \htmlinclude eeForceMsg.msg.html

(cl:defclass <eeForceMsg> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (jointAngles
    :reader jointAngles
    :initarg :jointAngles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass eeForceMsg (<eeForceMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eeForceMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eeForceMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee_force-msg:<eeForceMsg> is deprecated: use ee_force-msg:eeForceMsg instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <eeForceMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_force-msg:data-val is deprecated.  Use ee_force-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'jointAngles-val :lambda-list '(m))
(cl:defmethod jointAngles-val ((m <eeForceMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_force-msg:jointAngles-val is deprecated.  Use ee_force-msg:jointAngles instead.")
  (jointAngles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eeForceMsg>) ostream)
  "Serializes a message object of type '<eeForceMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'jointAngles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'jointAngles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eeForceMsg>) istream)
  "Deserializes a message object of type '<eeForceMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'jointAngles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'jointAngles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eeForceMsg>)))
  "Returns string type for a message object of type '<eeForceMsg>"
  "ee_force/eeForceMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eeForceMsg)))
  "Returns string type for a message object of type 'eeForceMsg"
  "ee_force/eeForceMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eeForceMsg>)))
  "Returns md5sum for a message object of type '<eeForceMsg>"
  "de54ba17451137cbb1bc233fa1cbaeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eeForceMsg)))
  "Returns md5sum for a message object of type 'eeForceMsg"
  "de54ba17451137cbb1bc233fa1cbaeed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eeForceMsg>)))
  "Returns full string definition for message of type '<eeForceMsg>"
  (cl:format cl:nil "float64[] data~%float64[] jointAngles~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eeForceMsg)))
  "Returns full string definition for message of type 'eeForceMsg"
  (cl:format cl:nil "float64[] data~%float64[] jointAngles~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eeForceMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'jointAngles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eeForceMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'eeForceMsg
    (cl:cons ':data (data msg))
    (cl:cons ':jointAngles (jointAngles msg))
))
