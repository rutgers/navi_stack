; Auto-generated. Do not edit!


(cl:in-package wiimote-msg)


;//! \htmlinclude LEDControl.msg.html

(cl:defclass <LEDControl> (roslisp-msg-protocol:ros-message)
  ((timed_switch_array
    :reader timed_switch_array
    :initarg :timed_switch_array
    :type (cl:vector wiimote-msg:TimedSwitch)
   :initform (cl:make-array 0 :element-type 'wiimote-msg:TimedSwitch :initial-element (cl:make-instance 'wiimote-msg:TimedSwitch))))
)

(cl:defclass LEDControl (<LEDControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LEDControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LEDControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wiimote-msg:<LEDControl> is deprecated: use wiimote-msg:LEDControl instead.")))

(cl:ensure-generic-function 'timed_switch_array-val :lambda-list '(m))
(cl:defmethod timed_switch_array-val ((m <LEDControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wiimote-msg:timed_switch_array-val is deprecated.  Use wiimote-msg:timed_switch_array instead.")
  (timed_switch_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LEDControl>) ostream)
  "Serializes a message object of type '<LEDControl>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timed_switch_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'timed_switch_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LEDControl>) istream)
  "Deserializes a message object of type '<LEDControl>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timed_switch_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timed_switch_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'wiimote-msg:TimedSwitch))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LEDControl>)))
  "Returns string type for a message object of type '<LEDControl>"
  "wiimote/LEDControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LEDControl)))
  "Returns string type for a message object of type 'LEDControl"
  "wiimote/LEDControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LEDControl>)))
  "Returns md5sum for a message object of type '<LEDControl>"
  "d0477eee2c164b8a9582a596f92c6f08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LEDControl)))
  "Returns md5sum for a message object of type 'LEDControl"
  "d0477eee2c164b8a9582a596f92c6f08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LEDControl>)))
  "Returns full string definition for message of type '<LEDControl>"
  (cl:format cl:nil "# Message to request Wiimote LED operations. The Wiimote has four~%# LEDs. This message enables its user to turn each individual LED~%# on or off, and to define a blink pattern for each LEDs.~%#        See TimedSwitch.msg for details.~%~%TimedSwitch[] timed_switch_array  ~%~%================================================================================~%MSG: wiimote/TimedSwitch~%# TimedSwitch allows sender to:~%#    o turn a switch on,~%#    o turn a switch off, and~%#    o repeat an on/off pattern forever or for a~%#          given number of times.~%# Fields (refer to definitions of constants in the definition body):~%#     o switch_mode:~%#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)~%#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)~%#  NO_CHANGE: leave LED in its current state~%#     REPEAT: repeat an on/off pattern for as long~%#             as is indicated in the num_cycles field. The~%#             pattern is defined in the pulse_pattern field.~%#~%#     o num_cycles:~%#          n>=0: run the pattern that is defined in pulse_pattern~%#                n times.~%#          n==FOREVER: run the pattern that is defined in pulse_pattern~%#                       until a new TimedSwitch message is sent.              ~%#~%#     o pulse_pattern:~%#          A series of time durations in fractions of a second. The~%#          first number is the duration for having the switch on.~%#          The second number is the duration for which the switch~%#          is off. The third is an 'on' period again, etc.~%#          A pattern is terminated with the end of the array.~%#           ~%#          Example: [1,1] specifies an on-off sequence of 1 second.               ~%~%int8 ON        =  1~%int8 OFF       =  0~%int8 NO_CHANGE = -2~%int8 REPEAT    = -1~%int8 FOREVER   = -1~%~%int8 switch_mode~%int32 num_cycles~%float32[] pulse_pattern~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LEDControl)))
  "Returns full string definition for message of type 'LEDControl"
  (cl:format cl:nil "# Message to request Wiimote LED operations. The Wiimote has four~%# LEDs. This message enables its user to turn each individual LED~%# on or off, and to define a blink pattern for each LEDs.~%#        See TimedSwitch.msg for details.~%~%TimedSwitch[] timed_switch_array  ~%~%================================================================================~%MSG: wiimote/TimedSwitch~%# TimedSwitch allows sender to:~%#    o turn a switch on,~%#    o turn a switch off, and~%#    o repeat an on/off pattern forever or for a~%#          given number of times.~%# Fields (refer to definitions of constants in the definition body):~%#     o switch_mode:~%#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)~%#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)~%#  NO_CHANGE: leave LED in its current state~%#     REPEAT: repeat an on/off pattern for as long~%#             as is indicated in the num_cycles field. The~%#             pattern is defined in the pulse_pattern field.~%#~%#     o num_cycles:~%#          n>=0: run the pattern that is defined in pulse_pattern~%#                n times.~%#          n==FOREVER: run the pattern that is defined in pulse_pattern~%#                       until a new TimedSwitch message is sent.              ~%#~%#     o pulse_pattern:~%#          A series of time durations in fractions of a second. The~%#          first number is the duration for having the switch on.~%#          The second number is the duration for which the switch~%#          is off. The third is an 'on' period again, etc.~%#          A pattern is terminated with the end of the array.~%#           ~%#          Example: [1,1] specifies an on-off sequence of 1 second.               ~%~%int8 ON        =  1~%int8 OFF       =  0~%int8 NO_CHANGE = -2~%int8 REPEAT    = -1~%int8 FOREVER   = -1~%~%int8 switch_mode~%int32 num_cycles~%float32[] pulse_pattern~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LEDControl>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timed_switch_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LEDControl>))
  "Converts a ROS message object to a list"
  (cl:list 'LEDControl
    (cl:cons ':timed_switch_array (timed_switch_array msg))
))
