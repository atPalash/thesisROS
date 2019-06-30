;; Auto-generated. Do not edit!


(when (boundp 'thesis_realsense::GripperData)
  (if (not (find-package "THESIS_REALSENSE"))
    (make-package "THESIS_REALSENSE"))
  (shadow 'GripperData (find-package "THESIS_REALSENSE")))
(unless (find-package "THESIS_REALSENSE::GRIPPERDATA")
  (make-package "THESIS_REALSENSE::GRIPPERDATA"))
(unless (find-package "THESIS_REALSENSE::GRIPPERDATAREQUEST")
  (make-package "THESIS_REALSENSE::GRIPPERDATAREQUEST"))
(unless (find-package "THESIS_REALSENSE::GRIPPERDATARESPONSE")
  (make-package "THESIS_REALSENSE::GRIPPERDATARESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass thesis_realsense::GripperDataRequest
  :super ros::object
  :slots (_id _grasp_pose _max_contact_force _max_contact_velocity _max_contact_width _release_pose ))

(defmethod thesis_realsense::GripperDataRequest
  (:init
   (&key
    ((:id __id) "")
    ((:grasp_pose __grasp_pose) (instance geometry_msgs::PoseStamped :init))
    ((:max_contact_force __max_contact_force) 0.0)
    ((:max_contact_velocity __max_contact_velocity) 0.0)
    ((:max_contact_width __max_contact_width) 0.0)
    ((:release_pose __release_pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _grasp_pose __grasp_pose)
   (setq _max_contact_force (float __max_contact_force))
   (setq _max_contact_velocity (float __max_contact_velocity))
   (setq _max_contact_width (float __max_contact_width))
   (setq _release_pose __release_pose)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:grasp_pose
   (&rest __grasp_pose)
   (if (keywordp (car __grasp_pose))
       (send* _grasp_pose __grasp_pose)
     (progn
       (if __grasp_pose (setq _grasp_pose (car __grasp_pose)))
       _grasp_pose)))
  (:max_contact_force
   (&optional __max_contact_force)
   (if __max_contact_force (setq _max_contact_force __max_contact_force)) _max_contact_force)
  (:max_contact_velocity
   (&optional __max_contact_velocity)
   (if __max_contact_velocity (setq _max_contact_velocity __max_contact_velocity)) _max_contact_velocity)
  (:max_contact_width
   (&optional __max_contact_width)
   (if __max_contact_width (setq _max_contact_width __max_contact_width)) _max_contact_width)
  (:release_pose
   (&rest __release_pose)
   (if (keywordp (car __release_pose))
       (send* _release_pose __release_pose)
     (progn
       (if __release_pose (setq _release_pose (car __release_pose)))
       _release_pose)))
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; geometry_msgs/PoseStamped _grasp_pose
    (send _grasp_pose :serialization-length)
    ;; float32 _max_contact_force
    4
    ;; float32 _max_contact_velocity
    4
    ;; float32 _max_contact_width
    4
    ;; geometry_msgs/PoseStamped _release_pose
    (send _release_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; geometry_msgs/PoseStamped _grasp_pose
       (send _grasp_pose :serialize s)
     ;; float32 _max_contact_force
       (sys::poke _max_contact_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_contact_velocity
       (sys::poke _max_contact_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_contact_width
       (sys::poke _max_contact_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/PoseStamped _release_pose
       (send _release_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseStamped _grasp_pose
     (send _grasp_pose :deserialize buf ptr-) (incf ptr- (send _grasp_pose :serialization-length))
   ;; float32 _max_contact_force
     (setq _max_contact_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_contact_velocity
     (setq _max_contact_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_contact_width
     (setq _max_contact_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/PoseStamped _release_pose
     (send _release_pose :deserialize buf ptr-) (incf ptr- (send _release_pose :serialization-length))
   ;;
   self)
  )

(defclass thesis_realsense::GripperDataResponse
  :super ros::object
  :slots (_grasp_result ))

(defmethod thesis_realsense::GripperDataResponse
  (:init
   (&key
    ((:grasp_result __grasp_result) nil)
    )
   (send-super :init)
   (setq _grasp_result __grasp_result)
   self)
  (:grasp_result
   (&optional __grasp_result)
   (if __grasp_result (setq _grasp_result __grasp_result)) _grasp_result)
  (:serialization-length
   ()
   (+
    ;; bool _grasp_result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _grasp_result
       (if _grasp_result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _grasp_result
     (setq _grasp_result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass thesis_realsense::GripperData
  :super ros::object
  :slots ())

(setf (get thesis_realsense::GripperData :md5sum-) "d34785af14fea5ec34d952bf17a99f4a")
(setf (get thesis_realsense::GripperData :datatype-) "thesis_realsense/GripperData")
(setf (get thesis_realsense::GripperData :request) thesis_realsense::GripperDataRequest)
(setf (get thesis_realsense::GripperData :response) thesis_realsense::GripperDataResponse)

(defmethod thesis_realsense::GripperDataRequest
  (:response () (instance thesis_realsense::GripperDataResponse :init)))

(setf (get thesis_realsense::GripperDataRequest :md5sum-) "d34785af14fea5ec34d952bf17a99f4a")
(setf (get thesis_realsense::GripperDataRequest :datatype-) "thesis_realsense/GripperDataRequest")
(setf (get thesis_realsense::GripperDataRequest :definition-)
      "
string id





geometry_msgs/PoseStamped grasp_pose


float32 max_contact_force


float32 max_contact_velocity


float32 max_contact_width





geometry_msgs/PoseStamped release_pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

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
---
bool grasp_result
")

(setf (get thesis_realsense::GripperDataResponse :md5sum-) "d34785af14fea5ec34d952bf17a99f4a")
(setf (get thesis_realsense::GripperDataResponse :datatype-) "thesis_realsense/GripperDataResponse")
(setf (get thesis_realsense::GripperDataResponse :definition-)
      "
string id





geometry_msgs/PoseStamped grasp_pose


float32 max_contact_force


float32 max_contact_velocity


float32 max_contact_width





geometry_msgs/PoseStamped release_pose

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
# 0: no frame
# 1: global frame
string frame_id

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
---
bool grasp_result
")



(provide :thesis_realsense/GripperData "d34785af14fea5ec34d952bf17a99f4a")


