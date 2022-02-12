; Auto-generated. Do not edit!


(cl:in-package calibration-srv)


;//! \htmlinclude SetJointPositions-request.msg.html

(cl:defclass <SetJointPositions-request> (roslisp-msg-protocol:ros-message)
  ((joint_positions
    :reader joint_positions
    :initarg :joint_positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetJointPositions-request (<SetJointPositions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointPositions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointPositions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calibration-srv:<SetJointPositions-request> is deprecated: use calibration-srv:SetJointPositions-request instead.")))

(cl:ensure-generic-function 'joint_positions-val :lambda-list '(m))
(cl:defmethod joint_positions-val ((m <SetJointPositions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-srv:joint_positions-val is deprecated.  Use calibration-srv:joint_positions instead.")
  (joint_positions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointPositions-request>) ostream)
  "Serializes a message object of type '<SetJointPositions-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointPositions-request>) istream)
  "Deserializes a message object of type '<SetJointPositions-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointPositions-request>)))
  "Returns string type for a service object of type '<SetJointPositions-request>"
  "calibration/SetJointPositionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPositions-request)))
  "Returns string type for a service object of type 'SetJointPositions-request"
  "calibration/SetJointPositionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointPositions-request>)))
  "Returns md5sum for a message object of type '<SetJointPositions-request>"
  "702c3dade1da9e295e178fcc32cce64f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointPositions-request)))
  "Returns md5sum for a message object of type 'SetJointPositions-request"
  "702c3dade1da9e295e178fcc32cce64f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointPositions-request>)))
  "Returns full string definition for message of type '<SetJointPositions-request>"
  (cl:format cl:nil "float64[] joint_positions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointPositions-request)))
  "Returns full string definition for message of type 'SetJointPositions-request"
  (cl:format cl:nil "float64[] joint_positions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointPositions-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointPositions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointPositions-request
    (cl:cons ':joint_positions (joint_positions msg))
))
;//! \htmlinclude SetJointPositions-response.msg.html

(cl:defclass <SetJointPositions-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetJointPositions-response (<SetJointPositions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetJointPositions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetJointPositions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calibration-srv:<SetJointPositions-response> is deprecated: use calibration-srv:SetJointPositions-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetJointPositions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-srv:result-val is deprecated.  Use calibration-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetJointPositions-response>) ostream)
  "Serializes a message object of type '<SetJointPositions-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetJointPositions-response>) istream)
  "Deserializes a message object of type '<SetJointPositions-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetJointPositions-response>)))
  "Returns string type for a service object of type '<SetJointPositions-response>"
  "calibration/SetJointPositionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPositions-response)))
  "Returns string type for a service object of type 'SetJointPositions-response"
  "calibration/SetJointPositionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetJointPositions-response>)))
  "Returns md5sum for a message object of type '<SetJointPositions-response>"
  "702c3dade1da9e295e178fcc32cce64f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetJointPositions-response)))
  "Returns md5sum for a message object of type 'SetJointPositions-response"
  "702c3dade1da9e295e178fcc32cce64f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetJointPositions-response>)))
  "Returns full string definition for message of type '<SetJointPositions-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetJointPositions-response)))
  "Returns full string definition for message of type 'SetJointPositions-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetJointPositions-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetJointPositions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetJointPositions-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetJointPositions)))
  'SetJointPositions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetJointPositions)))
  'SetJointPositions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetJointPositions)))
  "Returns string type for a service object of type '<SetJointPositions>"
  "calibration/SetJointPositions")