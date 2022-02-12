; Auto-generated. Do not edit!


(cl:in-package calibration-srv)


;//! \htmlinclude GetJointPositions-request.msg.html

(cl:defclass <GetJointPositions-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetJointPositions-request (<GetJointPositions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPositions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPositions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calibration-srv:<GetJointPositions-request> is deprecated: use calibration-srv:GetJointPositions-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPositions-request>) ostream)
  "Serializes a message object of type '<GetJointPositions-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPositions-request>) istream)
  "Deserializes a message object of type '<GetJointPositions-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPositions-request>)))
  "Returns string type for a service object of type '<GetJointPositions-request>"
  "calibration/GetJointPositionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPositions-request)))
  "Returns string type for a service object of type 'GetJointPositions-request"
  "calibration/GetJointPositionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPositions-request>)))
  "Returns md5sum for a message object of type '<GetJointPositions-request>"
  "a286ff40b196573b9ebf3999a2f8d438")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPositions-request)))
  "Returns md5sum for a message object of type 'GetJointPositions-request"
  "a286ff40b196573b9ebf3999a2f8d438")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPositions-request>)))
  "Returns full string definition for message of type '<GetJointPositions-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPositions-request)))
  "Returns full string definition for message of type 'GetJointPositions-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPositions-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPositions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPositions-request
))
;//! \htmlinclude GetJointPositions-response.msg.html

(cl:defclass <GetJointPositions-response> (roslisp-msg-protocol:ros-message)
  ((joint_positions
    :reader joint_positions
    :initarg :joint_positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetJointPositions-response (<GetJointPositions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointPositions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointPositions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calibration-srv:<GetJointPositions-response> is deprecated: use calibration-srv:GetJointPositions-response instead.")))

(cl:ensure-generic-function 'joint_positions-val :lambda-list '(m))
(cl:defmethod joint_positions-val ((m <GetJointPositions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-srv:joint_positions-val is deprecated.  Use calibration-srv:joint_positions instead.")
  (joint_positions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointPositions-response>) ostream)
  "Serializes a message object of type '<GetJointPositions-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_positions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_positions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointPositions-response>) istream)
  "Deserializes a message object of type '<GetJointPositions-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_positions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_positions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointPositions-response>)))
  "Returns string type for a service object of type '<GetJointPositions-response>"
  "calibration/GetJointPositionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPositions-response)))
  "Returns string type for a service object of type 'GetJointPositions-response"
  "calibration/GetJointPositionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointPositions-response>)))
  "Returns md5sum for a message object of type '<GetJointPositions-response>"
  "a286ff40b196573b9ebf3999a2f8d438")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointPositions-response)))
  "Returns md5sum for a message object of type 'GetJointPositions-response"
  "a286ff40b196573b9ebf3999a2f8d438")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointPositions-response>)))
  "Returns full string definition for message of type '<GetJointPositions-response>"
  (cl:format cl:nil "float64[] joint_positions~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointPositions-response)))
  "Returns full string definition for message of type 'GetJointPositions-response"
  (cl:format cl:nil "float64[] joint_positions~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointPositions-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointPositions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointPositions-response
    (cl:cons ':joint_positions (joint_positions msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointPositions)))
  'GetJointPositions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointPositions)))
  'GetJointPositions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointPositions)))
  "Returns string type for a service object of type '<GetJointPositions>"
  "calibration/GetJointPositions")