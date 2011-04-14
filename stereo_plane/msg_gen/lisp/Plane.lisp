; Auto-generated. Do not edit!


(cl:in-package stereo_plane-msg)


;//! \htmlinclude Plane.msg.html

(cl:defclass <Plane> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (normal
    :reader normal
    :initarg :normal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Plane (<Plane>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plane>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plane)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stereo_plane-msg:<Plane> is deprecated: use stereo_plane-msg:Plane instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Plane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stereo_plane-msg:header-val is deprecated.  Use stereo_plane-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Plane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stereo_plane-msg:point-val is deprecated.  Use stereo_plane-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'normal-val :lambda-list '(m))
(cl:defmethod normal-val ((m <Plane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stereo_plane-msg:normal-val is deprecated.  Use stereo_plane-msg:normal instead.")
  (normal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plane>) ostream)
  "Serializes a message object of type '<Plane>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'normal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plane>) istream)
  "Deserializes a message object of type '<Plane>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'normal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plane>)))
  "Returns string type for a message object of type '<Plane>"
  "stereo_plane/Plane")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plane)))
  "Returns string type for a message object of type 'Plane"
  "stereo_plane/Plane")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plane>)))
  "Returns md5sum for a message object of type '<Plane>"
  "e6dad39e8527110ab026c6017616f37b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plane)))
  "Returns md5sum for a message object of type 'Plane"
  "e6dad39e8527110ab026c6017616f37b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plane>)))
  "Returns full string definition for message of type '<Plane>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point   point~%geometry_msgs/Vector3 normal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plane)))
  "Returns full string definition for message of type 'Plane"
  (cl:format cl:nil "Header header~%geometry_msgs/Point   point~%geometry_msgs/Vector3 normal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plane>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'normal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plane>))
  "Converts a ROS message object to a list"
  (cl:list 'Plane
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
    (cl:cons ':normal (normal msg))
))
