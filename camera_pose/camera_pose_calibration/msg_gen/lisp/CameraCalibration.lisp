; Auto-generated. Do not edit!


(cl:in-package camera_pose_calibration-msg)


;//! \htmlinclude CameraCalibration.msg.html

(cl:defclass <CameraCalibration> (roslisp-msg-protocol:ros-message)
  ((camera_pose
    :reader camera_pose
    :initarg :camera_pose
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (camera_id
    :reader camera_id
    :initarg :camera_id
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass CameraCalibration (<CameraCalibration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraCalibration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraCalibration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_pose_calibration-msg:<CameraCalibration> is deprecated: use camera_pose_calibration-msg:CameraCalibration instead.")))

(cl:ensure-generic-function 'camera_pose-val :lambda-list '(m))
(cl:defmethod camera_pose-val ((m <CameraCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:camera_pose-val is deprecated.  Use camera_pose_calibration-msg:camera_pose instead.")
  (camera_pose m))

(cl:ensure-generic-function 'camera_id-val :lambda-list '(m))
(cl:defmethod camera_id-val ((m <CameraCalibration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_pose_calibration-msg:camera_id-val is deprecated.  Use camera_pose_calibration-msg:camera_id instead.")
  (camera_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraCalibration>) ostream)
  "Serializes a message object of type '<CameraCalibration>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'camera_pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'camera_pose))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'camera_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'camera_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraCalibration>) istream)
  "Deserializes a message object of type '<CameraCalibration>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'camera_pose) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'camera_pose)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'camera_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'camera_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraCalibration>)))
  "Returns string type for a message object of type '<CameraCalibration>"
  "camera_pose_calibration/CameraCalibration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraCalibration)))
  "Returns string type for a message object of type 'CameraCalibration"
  "camera_pose_calibration/CameraCalibration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraCalibration>)))
  "Returns md5sum for a message object of type '<CameraCalibration>"
  "7c56b4c541b0941c246e565a97eb5388")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraCalibration)))
  "Returns md5sum for a message object of type 'CameraCalibration"
  "7c56b4c541b0941c246e565a97eb5388")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraCalibration>)))
  "Returns full string definition for message of type '<CameraCalibration>"
  (cl:format cl:nil "geometry_msgs/Pose[] camera_pose~%string[] camera_id~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraCalibration)))
  "Returns full string definition for message of type 'CameraCalibration"
  (cl:format cl:nil "geometry_msgs/Pose[] camera_pose~%string[] camera_id~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraCalibration>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'camera_pose) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'camera_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraCalibration>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraCalibration
    (cl:cons ':camera_pose (camera_pose msg))
    (cl:cons ':camera_id (camera_id msg))
))
