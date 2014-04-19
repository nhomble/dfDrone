; Auto-generated. Do not edit!


(cl:in-package dfDrone-msg)


;//! \htmlinclude DFDVelocity.msg.html

(cl:defclass <DFDVelocity> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass DFDVelocity (<DFDVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DFDVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DFDVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dfDrone-msg:<DFDVelocity> is deprecated: use dfDrone-msg:DFDVelocity instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DFDVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dfDrone-msg:header-val is deprecated.  Use dfDrone-msg:header instead.")
  (header m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DFDVelocity>) ostream)
  "Serializes a message object of type '<DFDVelocity>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DFDVelocity>) istream)
  "Deserializes a message object of type '<DFDVelocity>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DFDVelocity>)))
  "Returns string type for a message object of type '<DFDVelocity>"
  "dfDrone/DFDVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DFDVelocity)))
  "Returns string type for a message object of type 'DFDVelocity"
  "dfDrone/DFDVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DFDVelocity>)))
  "Returns md5sum for a message object of type '<DFDVelocity>"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DFDVelocity)))
  "Returns md5sum for a message object of type 'DFDVelocity"
  "d7be0bb39af8fb9129d5a76e6b63a290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DFDVelocity>)))
  "Returns full string definition for message of type '<DFDVelocity>"
  (cl:format cl:nil "Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DFDVelocity)))
  "Returns full string definition for message of type 'DFDVelocity"
  (cl:format cl:nil "Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DFDVelocity>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DFDVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'DFDVelocity
    (cl:cons ':header (header msg))
))
