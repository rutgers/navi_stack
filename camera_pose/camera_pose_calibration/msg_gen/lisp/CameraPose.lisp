; Auto-generated. Do not edit!


(cl:in-package camera_pose_calibration-msg)


;//! \htmlinclude CameraPose.msg.html

(cl:defclass <CameraPose> (roslisp-msg-protocol:ros-message)
  ((camera_id
    :reader camera_id
    :initarg :camera_id
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass CameraPose (<CameraPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_pose_calibration-msg:<CameraPose> is deprecated: use camera_pose_calibration-msg:CameraPose instead.")))

(cl:ensure-generic-function 'camera_id-val :lambda-list '(m))
(cl:defmethod camera_id-val ((m <CameraPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:camera_id-val is deprecated.  Use camera_pose_calibration-msg:camera_id instead.")
  (camera_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <CameraPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:pose-val is deprecated.  Use camera_pose_calibration-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraPose>) ostream)
  "Serializes a message object of type '<CameraPose>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camera_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camera_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraPose>) istream)
  "Deserializes a message object of type '<CameraPose>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camera_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraPose>)))
  "Returns string type for a message object of type '<CameraPose>"
  "camera_pose_calibration/CameraPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraPose)))
  "Returns string type for a message object of type 'CameraPose"
  "camera_pose_calibration/CameraPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraPose>)))
  "Returns md5sum for a message object of type '<CameraPose>"
  "eb4c53d8c0c861e2c5d562bd921bb38e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraPose)))
  "Returns md5sum for a message object of type 'CameraPose"
  "eb4c53d8c0c861e2c5d562bd921bb38e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraPose>)))
  "Returns full string definition for message of type '<CameraPose>"
  (cl:format cl:nil "string camera_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraPose)))
  "Returns full string definition for message of type 'CameraPose"
  (cl:format cl:nil "string camera_id~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraPose>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'camera_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraPose>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraPose
    (cl:cons ':camera_id (camera_id msg))
    (cl:cons ':pose (pose msg))
))
