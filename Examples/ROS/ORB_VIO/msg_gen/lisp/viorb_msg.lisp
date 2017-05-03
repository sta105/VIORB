; Auto-generated. Do not edit!


(cl:in-package ORB_VIO-msg)


;//! \htmlinclude viorb_msg.msg.html

(cl:defclass <viorb_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Tic
    :reader Tic
    :initarg :Tic
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (Qwi
    :reader Qwi
    :initarg :Qwi
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (gw
    :reader gw
    :initarg :gw
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (VINSInitFlag
    :reader VINSInitFlag
    :initarg :VINSInitFlag
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (TrackStatus
    :reader TrackStatus
    :initarg :TrackStatus
    :type std_msgs-msg:Int8
    :initform (cl:make-instance 'std_msgs-msg:Int8)))
)

(cl:defclass viorb_msg (<viorb_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <viorb_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'viorb_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ORB_VIO-msg:<viorb_msg> is deprecated: use ORB_VIO-msg:viorb_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:header-val is deprecated.  Use ORB_VIO-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Tic-val :lambda-list '(m))
(cl:defmethod Tic-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:Tic-val is deprecated.  Use ORB_VIO-msg:Tic instead.")
  (Tic m))

(cl:ensure-generic-function 'Qwi-val :lambda-list '(m))
(cl:defmethod Qwi-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:Qwi-val is deprecated.  Use ORB_VIO-msg:Qwi instead.")
  (Qwi m))

(cl:ensure-generic-function 'gw-val :lambda-list '(m))
(cl:defmethod gw-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:gw-val is deprecated.  Use ORB_VIO-msg:gw instead.")
  (gw m))

(cl:ensure-generic-function 'VINSInitFlag-val :lambda-list '(m))
(cl:defmethod VINSInitFlag-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:VINSInitFlag-val is deprecated.  Use ORB_VIO-msg:VINSInitFlag instead.")
  (VINSInitFlag m))

(cl:ensure-generic-function 'TrackStatus-val :lambda-list '(m))
(cl:defmethod TrackStatus-val ((m <viorb_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ORB_VIO-msg:TrackStatus-val is deprecated.  Use ORB_VIO-msg:TrackStatus instead.")
  (TrackStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <viorb_msg>) ostream)
  "Serializes a message object of type '<viorb_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Tic) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Qwi) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gw) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'VINSInitFlag) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'TrackStatus) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <viorb_msg>) istream)
  "Deserializes a message object of type '<viorb_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Tic) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Qwi) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gw) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'VINSInitFlag) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'TrackStatus) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<viorb_msg>)))
  "Returns string type for a message object of type '<viorb_msg>"
  "ORB_VIO/viorb_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'viorb_msg)))
  "Returns string type for a message object of type 'viorb_msg"
  "ORB_VIO/viorb_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<viorb_msg>)))
  "Returns md5sum for a message object of type '<viorb_msg>"
  "22701a4895958bd1b68178837a5aa9f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'viorb_msg)))
  "Returns md5sum for a message object of type 'viorb_msg"
  "22701a4895958bd1b68178837a5aa9f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<viorb_msg>)))
  "Returns full string definition for message of type '<viorb_msg>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose Tic~%geometry_msgs/Quaternion Qwi~%geometry_msgs/Point gw~%std_msgs/Bool VINSInitFlag~%std_msgs/Int8 TrackStatus~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'viorb_msg)))
  "Returns full string definition for message of type 'viorb_msg"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose Tic~%geometry_msgs/Quaternion Qwi~%geometry_msgs/Point gw~%std_msgs/Bool VINSInitFlag~%std_msgs/Int8 TrackStatus~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Int8~%int8 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <viorb_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Tic))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Qwi))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gw))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'VINSInitFlag))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'TrackStatus))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <viorb_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'viorb_msg
    (cl:cons ':header (header msg))
    (cl:cons ':Tic (Tic msg))
    (cl:cons ':Qwi (Qwi msg))
    (cl:cons ':gw (gw msg))
    (cl:cons ':VINSInitFlag (VINSInitFlag msg))
    (cl:cons ':TrackStatus (TrackStatus msg))
))
