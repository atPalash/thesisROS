; Auto-generated. Do not edit!


(cl:in-package thesis_realsense-srv)


;//! \htmlinclude GripperData-request.msg.html

(cl:defclass <GripperData-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (grasp_pose
    :reader grasp_pose
    :initarg :grasp_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (max_contact_force
    :reader max_contact_force
    :initarg :max_contact_force
    :type cl:float
    :initform 0.0)
   (max_contact_velocity
    :reader max_contact_velocity
    :initarg :max_contact_velocity
    :type cl:float
    :initform 0.0)
   (max_contact_width
    :reader max_contact_width
    :initarg :max_contact_width
    :type cl:float
    :initform 0.0)
   (release_pose
    :reader release_pose
    :initarg :release_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass GripperData-request (<GripperData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thesis_realsense-srv:<GripperData-request> is deprecated: use thesis_realsense-srv:GripperData-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:id-val is deprecated.  Use thesis_realsense-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'grasp_pose-val :lambda-list '(m))
(cl:defmethod grasp_pose-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:grasp_pose-val is deprecated.  Use thesis_realsense-srv:grasp_pose instead.")
  (grasp_pose m))

(cl:ensure-generic-function 'max_contact_force-val :lambda-list '(m))
(cl:defmethod max_contact_force-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:max_contact_force-val is deprecated.  Use thesis_realsense-srv:max_contact_force instead.")
  (max_contact_force m))

(cl:ensure-generic-function 'max_contact_velocity-val :lambda-list '(m))
(cl:defmethod max_contact_velocity-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:max_contact_velocity-val is deprecated.  Use thesis_realsense-srv:max_contact_velocity instead.")
  (max_contact_velocity m))

(cl:ensure-generic-function 'max_contact_width-val :lambda-list '(m))
(cl:defmethod max_contact_width-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:max_contact_width-val is deprecated.  Use thesis_realsense-srv:max_contact_width instead.")
  (max_contact_width m))

(cl:ensure-generic-function 'release_pose-val :lambda-list '(m))
(cl:defmethod release_pose-val ((m <GripperData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:release_pose-val is deprecated.  Use thesis_realsense-srv:release_pose instead.")
  (release_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperData-request>) ostream)
  "Serializes a message object of type '<GripperData-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_contact_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_contact_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_contact_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'release_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperData-request>) istream)
  "Deserializes a message object of type '<GripperData-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_contact_force) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_contact_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_contact_width) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'release_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperData-request>)))
  "Returns string type for a service object of type '<GripperData-request>"
  "thesis_realsense/GripperDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperData-request)))
  "Returns string type for a service object of type 'GripperData-request"
  "thesis_realsense/GripperDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperData-request>)))
  "Returns md5sum for a message object of type '<GripperData-request>"
  "d34785af14fea5ec34d952bf17a99f4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperData-request)))
  "Returns md5sum for a message object of type 'GripperData-request"
  "d34785af14fea5ec34d952bf17a99f4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperData-request>)))
  "Returns full string definition for message of type '<GripperData-request>"
  (cl:format cl:nil "~%string id~%~%~%~%~%~%geometry_msgs/PoseStamped grasp_pose~%~%~%float32 max_contact_force~%~%~%float32 max_contact_velocity~%~%~%float32 max_contact_width~%~%~%~%~%~%geometry_msgs/PoseStamped release_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperData-request)))
  "Returns full string definition for message of type 'GripperData-request"
  (cl:format cl:nil "~%string id~%~%~%~%~%~%geometry_msgs/PoseStamped grasp_pose~%~%~%float32 max_contact_force~%~%~%float32 max_contact_velocity~%~%~%float32 max_contact_width~%~%~%~%~%~%geometry_msgs/PoseStamped release_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperData-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp_pose))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'release_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperData-request
    (cl:cons ':id (id msg))
    (cl:cons ':grasp_pose (grasp_pose msg))
    (cl:cons ':max_contact_force (max_contact_force msg))
    (cl:cons ':max_contact_velocity (max_contact_velocity msg))
    (cl:cons ':max_contact_width (max_contact_width msg))
    (cl:cons ':release_pose (release_pose msg))
))
;//! \htmlinclude GripperData-response.msg.html

(cl:defclass <GripperData-response> (roslisp-msg-protocol:ros-message)
  ((grasp_result
    :reader grasp_result
    :initarg :grasp_result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GripperData-response (<GripperData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name thesis_realsense-srv:<GripperData-response> is deprecated: use thesis_realsense-srv:GripperData-response instead.")))

(cl:ensure-generic-function 'grasp_result-val :lambda-list '(m))
(cl:defmethod grasp_result-val ((m <GripperData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader thesis_realsense-srv:grasp_result-val is deprecated.  Use thesis_realsense-srv:grasp_result instead.")
  (grasp_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperData-response>) ostream)
  "Serializes a message object of type '<GripperData-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'grasp_result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperData-response>) istream)
  "Deserializes a message object of type '<GripperData-response>"
    (cl:setf (cl:slot-value msg 'grasp_result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperData-response>)))
  "Returns string type for a service object of type '<GripperData-response>"
  "thesis_realsense/GripperDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperData-response)))
  "Returns string type for a service object of type 'GripperData-response"
  "thesis_realsense/GripperDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperData-response>)))
  "Returns md5sum for a message object of type '<GripperData-response>"
  "d34785af14fea5ec34d952bf17a99f4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperData-response)))
  "Returns md5sum for a message object of type 'GripperData-response"
  "d34785af14fea5ec34d952bf17a99f4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperData-response>)))
  "Returns full string definition for message of type '<GripperData-response>"
  (cl:format cl:nil "bool grasp_result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperData-response)))
  "Returns full string definition for message of type 'GripperData-response"
  (cl:format cl:nil "bool grasp_result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperData-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperData-response
    (cl:cons ':grasp_result (grasp_result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperData)))
  'GripperData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperData)))
  'GripperData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperData)))
  "Returns string type for a service object of type '<GripperData>"
  "thesis_realsense/GripperData")