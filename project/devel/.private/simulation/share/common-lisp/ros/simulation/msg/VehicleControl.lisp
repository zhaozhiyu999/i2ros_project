; Auto-generated. Do not edit!


(cl:in-package simulation-msg)


;//! \htmlinclude VehicleControl.msg.html

(cl:defclass <VehicleControl> (roslisp-msg-protocol:ros-message)
  ((Throttle
    :reader Throttle
    :initarg :Throttle
    :type cl:float
    :initform 0.0)
   (Steering
    :reader Steering
    :initarg :Steering
    :type cl:float
    :initform 0.0)
   (Brake
    :reader Brake
    :initarg :Brake
    :type cl:float
    :initform 0.0)
   (Reserved
    :reader Reserved
    :initarg :Reserved
    :type cl:float
    :initform 0.0))
)

(cl:defclass VehicleControl (<VehicleControl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleControl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleControl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulation-msg:<VehicleControl> is deprecated: use simulation-msg:VehicleControl instead.")))

(cl:ensure-generic-function 'Throttle-val :lambda-list '(m))
(cl:defmethod Throttle-val ((m <VehicleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulation-msg:Throttle-val is deprecated.  Use simulation-msg:Throttle instead.")
  (Throttle m))

(cl:ensure-generic-function 'Steering-val :lambda-list '(m))
(cl:defmethod Steering-val ((m <VehicleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulation-msg:Steering-val is deprecated.  Use simulation-msg:Steering instead.")
  (Steering m))

(cl:ensure-generic-function 'Brake-val :lambda-list '(m))
(cl:defmethod Brake-val ((m <VehicleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulation-msg:Brake-val is deprecated.  Use simulation-msg:Brake instead.")
  (Brake m))

(cl:ensure-generic-function 'Reserved-val :lambda-list '(m))
(cl:defmethod Reserved-val ((m <VehicleControl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulation-msg:Reserved-val is deprecated.  Use simulation-msg:Reserved instead.")
  (Reserved m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleControl>) ostream)
  "Serializes a message object of type '<VehicleControl>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Throttle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Steering))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Brake))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Reserved))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleControl>) istream)
  "Deserializes a message object of type '<VehicleControl>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Throttle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Steering) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Brake) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Reserved) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleControl>)))
  "Returns string type for a message object of type '<VehicleControl>"
  "simulation/VehicleControl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleControl)))
  "Returns string type for a message object of type 'VehicleControl"
  "simulation/VehicleControl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleControl>)))
  "Returns md5sum for a message object of type '<VehicleControl>"
  "0d66040a38beac1d32107b24d8aabeae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleControl)))
  "Returns md5sum for a message object of type 'VehicleControl"
  "0d66040a38beac1d32107b24d8aabeae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleControl>)))
  "Returns full string definition for message of type '<VehicleControl>"
  (cl:format cl:nil "float32 Throttle~%float32 Steering~%float32 Brake~%float32 Reserved~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleControl)))
  "Returns full string definition for message of type 'VehicleControl"
  (cl:format cl:nil "float32 Throttle~%float32 Steering~%float32 Brake~%float32 Reserved~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleControl>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleControl>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleControl
    (cl:cons ':Throttle (Throttle msg))
    (cl:cons ':Steering (Steering msg))
    (cl:cons ':Brake (Brake msg))
    (cl:cons ':Reserved (Reserved msg))
))
