; Auto-generated. Do not edit!


(cl:in-package slide_box-srv)


;//! \htmlinclude robot_actuate_object-request.msg.html

(cl:defclass <robot_actuate_object-request> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass robot_actuate_object-request (<robot_actuate_object-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_actuate_object-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_actuate_object-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name slide_box-srv:<robot_actuate_object-request> is deprecated: use slide_box-srv:robot_actuate_object-request instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <robot_actuate_object-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slide_box-srv:action-val is deprecated.  Use slide_box-srv:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_actuate_object-request>) ostream)
  "Serializes a message object of type '<robot_actuate_object-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'action))))
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
   (cl:slot-value msg 'action))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_actuate_object-request>) istream)
  "Deserializes a message object of type '<robot_actuate_object-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'action) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'action)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_actuate_object-request>)))
  "Returns string type for a service object of type '<robot_actuate_object-request>"
  "slide_box/robot_actuate_objectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_actuate_object-request)))
  "Returns string type for a service object of type 'robot_actuate_object-request"
  "slide_box/robot_actuate_objectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_actuate_object-request>)))
  "Returns md5sum for a message object of type '<robot_actuate_object-request>"
  "b7dc8304fadbcfaba89fb93716c57805")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_actuate_object-request)))
  "Returns md5sum for a message object of type 'robot_actuate_object-request"
  "b7dc8304fadbcfaba89fb93716c57805")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_actuate_object-request>)))
  "Returns full string definition for message of type '<robot_actuate_object-request>"
  (cl:format cl:nil "float64[] action~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_actuate_object-request)))
  "Returns full string definition for message of type 'robot_actuate_object-request"
  (cl:format cl:nil "float64[] action~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_actuate_object-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'action) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_actuate_object-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_actuate_object-request
    (cl:cons ':action (action msg))
))
;//! \htmlinclude robot_actuate_object-response.msg.html

(cl:defclass <robot_actuate_object-response> (roslisp-msg-protocol:ros-message)
  ((obs
    :reader obs
    :initarg :obs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass robot_actuate_object-response (<robot_actuate_object-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_actuate_object-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_actuate_object-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name slide_box-srv:<robot_actuate_object-response> is deprecated: use slide_box-srv:robot_actuate_object-response instead.")))

(cl:ensure-generic-function 'obs-val :lambda-list '(m))
(cl:defmethod obs-val ((m <robot_actuate_object-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader slide_box-srv:obs-val is deprecated.  Use slide_box-srv:obs instead.")
  (obs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_actuate_object-response>) ostream)
  "Serializes a message object of type '<robot_actuate_object-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obs))))
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
   (cl:slot-value msg 'obs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_actuate_object-response>) istream)
  "Deserializes a message object of type '<robot_actuate_object-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obs)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_actuate_object-response>)))
  "Returns string type for a service object of type '<robot_actuate_object-response>"
  "slide_box/robot_actuate_objectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_actuate_object-response)))
  "Returns string type for a service object of type 'robot_actuate_object-response"
  "slide_box/robot_actuate_objectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_actuate_object-response>)))
  "Returns md5sum for a message object of type '<robot_actuate_object-response>"
  "b7dc8304fadbcfaba89fb93716c57805")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_actuate_object-response)))
  "Returns md5sum for a message object of type 'robot_actuate_object-response"
  "b7dc8304fadbcfaba89fb93716c57805")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_actuate_object-response>)))
  "Returns full string definition for message of type '<robot_actuate_object-response>"
  (cl:format cl:nil "float64[] obs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_actuate_object-response)))
  "Returns full string definition for message of type 'robot_actuate_object-response"
  (cl:format cl:nil "float64[] obs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_actuate_object-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_actuate_object-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_actuate_object-response
    (cl:cons ':obs (obs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_actuate_object)))
  'robot_actuate_object-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_actuate_object)))
  'robot_actuate_object-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_actuate_object)))
  "Returns string type for a service object of type '<robot_actuate_object>"
  "slide_box/robot_actuate_object")