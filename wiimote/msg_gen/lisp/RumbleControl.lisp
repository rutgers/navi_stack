; Auto-generated. Do not edit!


(cl:in-package wiimote-msg)


;//! \htmlinclude RumbleControl.msg.html

(cl:defclass <RumbleControl> (roslisp-msg-protocol:ros-message)
  ((rumble
    :reader rumble
    :initarg :rumble
    :type wiimote-msg:TimedSwitch
    :initform (cl:make-instance 'wiimote-msg:TimedSwitch)))
)

(cl:defclass RumbleControl (<RumbleControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RumbleControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RumbleControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wiimote-msg:<RumbleControl> is deprecated: use wiimote-msg:RumbleControl instead.")))

(cl:ensure-generic-function 'rumble-val :lambda-list '(m))
(cl:defmethod rumble-val ((m <RumbleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wiimote-msg:rumble-val is deprecated.  Use wiimote-msg:rumble instead.")
  (rumble m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RumbleControl>) ostream)
  "Serializes a message object of type '<RumbleControl>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rumble) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RumbleControl>) istream)
  "Deserializes a message object of type '<RumbleControl>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rumble) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RumbleControl>)))
  "Returns string type for a message object of type '<RumbleControl>"
  "wiimote/RumbleControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RumbleControl)))
  "Returns string type for a message object of type 'RumbleControl"
  "wiimote/RumbleControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RumbleControl>)))
  "Returns md5sum for a message object of type '<RumbleControl>"
  "264ec2f3013a512070cd67c42486214e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RumbleControl)))
  "Returns md5sum for a message object of type 'RumbleControl"
  "264ec2f3013a512070cd67c42486214e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RumbleControl>)))
  "Returns full string definition for message of type '<RumbleControl>"
  (cl:format cl:nil "# Message to control the Wiimote rumble (vibrator).~%# We simply use a TimedSwitch. So rumble can~%# be turned on (switch_mode == 1), off (switch_mode == 0),~%# or pulsed with a timing pattern (switch_mode == -1).~%# Number of times the cycle is repeated: num_cycles~%# (-1: repeat till next RumbleControl message). pulse_pattern~%# contains the time on/off pattern. (see also TimedSwitch.msg)~%~%TimedSwitch rumble~%~%~%================================================================================~%MSG: wiimote/TimedSwitch~%# TimedSwitch allows sender to:~%#    o turn a switch on,~%#    o turn a switch off, and~%#    o repeat an on/off pattern forever or for a~%#          given number of times.~%# Fields (refer to definitions of constants in the definition body):~%#     o switch_mode:~%#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)~%#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)~%#  NO_CHANGE: leave LED in its current state~%#     REPEAT: repeat an on/off pattern for as long~%#             as is indicated in the num_cycles field. The~%#             pattern is defined in the pulse_pattern field.~%#~%#     o num_cycles:~%#          n>=0: run the pattern that is defined in pulse_pattern~%#                n times.~%#          n==FOREVER: run the pattern that is defined in pulse_pattern~%#                       until a new TimedSwitch message is sent.              ~%#~%#     o pulse_pattern:~%#          A series of time durations in fractions of a second. The~%#          first number is the duration for having the switch on.~%#          The second number is the duration for which the switch~%#          is off. The third is an 'on' period again, etc.~%#          A pattern is terminated with the end of the array.~%#           ~%#          Example: [1,1] specifies an on-off sequence of 1 second.               ~%~%int8 ON        =  1~%int8 OFF       =  0~%int8 NO_CHANGE = -2~%int8 REPEAT    = -1~%int8 FOREVER   = -1~%~%int8 switch_mode~%int32 num_cycles~%float32[] pulse_pattern~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RumbleControl)))
  "Returns full string definition for message of type 'RumbleControl"
  (cl:format cl:nil "# Message to control the Wiimote rumble (vibrator).~%# We simply use a TimedSwitch. So rumble can~%# be turned on (switch_mode == 1), off (switch_mode == 0),~%# or pulsed with a timing pattern (switch_mode == -1).~%# Number of times the cycle is repeated: num_cycles~%# (-1: repeat till next RumbleControl message). pulse_pattern~%# contains the time on/off pattern. (see also TimedSwitch.msg)~%~%TimedSwitch rumble~%~%~%================================================================================~%MSG: wiimote/TimedSwitch~%# TimedSwitch allows sender to:~%#    o turn a switch on,~%#    o turn a switch off, and~%#    o repeat an on/off pattern forever or for a~%#          given number of times.~%# Fields (refer to definitions of constants in the definition body):~%#     o switch_mode:~%#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)~%#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)~%#  NO_CHANGE: leave LED in its current state~%#     REPEAT: repeat an on/off pattern for as long~%#             as is indicated in the num_cycles field. The~%#             pattern is defined in the pulse_pattern field.~%#~%#     o num_cycles:~%#          n>=0: run the pattern that is defined in pulse_pattern~%#                n times.~%#          n==FOREVER: run the pattern that is defined in pulse_pattern~%#                       until a new TimedSwitch message is sent.              ~%#~%#     o pulse_pattern:~%#          A series of time durations in fractions of a second. The~%#          first number is the duration for having the switch on.~%#          The second number is the duration for which the switch~%#          is off. The third is an 'on' period again, etc.~%#          A pattern is terminated with the end of the array.~%#           ~%#          Example: [1,1] specifies an on-off sequence of 1 second.               ~%~%int8 ON        =  1~%int8 OFF       =  0~%int8 NO_CHANGE = -2~%int8 REPEAT    = -1~%int8 FOREVER   = -1~%~%int8 switch_mode~%int32 num_cycles~%float32[] pulse_pattern~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RumbleControl>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rumble))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RumbleControl>))
  "Converts a ROS message object to a list"
  (cl:list 'RumbleControl
    (cl:cons ':rumble (rumble msg))
))
