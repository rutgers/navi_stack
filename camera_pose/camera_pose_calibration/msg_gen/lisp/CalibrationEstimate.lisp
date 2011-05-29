; Auto-generated. Do not edit!


(cl:in-package camera_pose_calibration-msg)


;//! \htmlinclude CalibrationEstimate.msg.html

(cl:defclass <CalibrationEstimate> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cameras
    :reader cameras
    :initarg :cameras
    :type (cl:vector camera_pose_calibration-msg:CameraPose)
   :initform (cl:make-array 0 :element-type 'camera_pose_calibration-msg:CameraPose :initial-element (cl:make-instance 'camera_pose_calibration-msg:CameraPose)))
   (targets
    :reader targets
    :initarg :targets
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass CalibrationEstimate (<CalibrationEstimate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationEstimate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationEstimate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_pose_calibration-msg:<CalibrationEstimate> is deprecated: use camera_pose_calibration-msg:CalibrationEstimate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CalibrationEstimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:header-val is deprecated.  Use camera_pose_calibration-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cameras-val :lambda-list '(m))
(cl:defmethod cameras-val ((m <CalibrationEstimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:cameras-val is deprecated.  Use camera_pose_calibration-msg:cameras instead.")
  (cameras m))

(cl:ensure-generic-function 'targets-val :lambda-list '(m))
(cl:defmethod targets-val ((m <CalibrationEstimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:targets-val is deprecated.  Use camera_pose_calibration-msg:targets instead.")
  (targets m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationEstimate>) ostream)
  "Serializes a message object of type '<CalibrationEstimate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cameras))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cameras))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'targets))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'targets))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationEstimate>) istream)
  "Deserializes a message object of type '<CalibrationEstimate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cameras) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cameras)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'camera_pose_calibration-msg:CameraPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'targets) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'targets)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationEstimate>)))
  "Returns string type for a message object of type '<CalibrationEstimate>"
  "camera_pose_calibration/CalibrationEstimate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationEstimate)))
  "Returns string type for a message object of type 'CalibrationEstimate"
  "camera_pose_calibration/CalibrationEstimate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationEstimate>)))
  "Returns md5sum for a message object of type '<CalibrationEstimate>"
  "35dff6bdb713fbc8b83861a1960b45f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationEstimate)))
  "Returns md5sum for a message object of type 'CalibrationEstimate"
  "35dff6bdb713fbc8b83861a1960b45f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationEstimate>)))
  "Returns full string definition for message of type '<CalibrationEstimate>"
  (cl:format cl:nil "Header header~%CameraPose[] cameras~%geometry_msgs/Pose[] targets~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: camera_pose_calibration/CameraPose~%string camera_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationEstimate)))
  "Returns full string definition for message of type 'CalibrationEstimate"
  (cl:format cl:nil "Header header~%CameraPose[] cameras~%geometry_msgs/Pose[] targets~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: camera_pose_calibration/CameraPose~%string camera_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationEstimate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cameras) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targets) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationEstimate>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationEstimate
    (cl:cons ':header (header msg))
    (cl:cons ':cameras (cameras msg))
    (cl:cons ':targets (targets msg))
))
