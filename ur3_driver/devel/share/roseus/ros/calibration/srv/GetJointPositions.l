;; Auto-generated. Do not edit!


(when (boundp 'calibration::GetJointPositions)
  (if (not (find-package "CALIBRATION"))
    (make-package "CALIBRATION"))
  (shadow 'GetJointPositions (find-package "CALIBRATION")))
(unless (find-package "CALIBRATION::GETJOINTPOSITIONS")
  (make-package "CALIBRATION::GETJOINTPOSITIONS"))
(unless (find-package "CALIBRATION::GETJOINTPOSITIONSREQUEST")
  (make-package "CALIBRATION::GETJOINTPOSITIONSREQUEST"))
(unless (find-package "CALIBRATION::GETJOINTPOSITIONSRESPONSE")
  (make-package "CALIBRATION::GETJOINTPOSITIONSRESPONSE"))

(in-package "ROS")





(defclass calibration::GetJointPositionsRequest
  :super ros::object
  :slots ())

(defmethod calibration::GetJointPositionsRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass calibration::GetJointPositionsResponse
  :super ros::object
  :slots (_joint_positions ))

(defmethod calibration::GetJointPositionsResponse
  (:init
   (&key
    ((:joint_positions __joint_positions) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_positions __joint_positions)
   self)
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_positions
    (* 8    (length _joint_positions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_positions
     (write-long (length _joint_positions) s)
     (dotimes (i (length _joint_positions))
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass calibration::GetJointPositions
  :super ros::object
  :slots ())

(setf (get calibration::GetJointPositions :md5sum-) "a286ff40b196573b9ebf3999a2f8d438")
(setf (get calibration::GetJointPositions :datatype-) "calibration/GetJointPositions")
(setf (get calibration::GetJointPositions :request) calibration::GetJointPositionsRequest)
(setf (get calibration::GetJointPositions :response) calibration::GetJointPositionsResponse)

(defmethod calibration::GetJointPositionsRequest
  (:response () (instance calibration::GetJointPositionsResponse :init)))

(setf (get calibration::GetJointPositionsRequest :md5sum-) "a286ff40b196573b9ebf3999a2f8d438")
(setf (get calibration::GetJointPositionsRequest :datatype-) "calibration/GetJointPositionsRequest")
(setf (get calibration::GetJointPositionsRequest :definition-)
      "---
float64[] joint_positions

")

(setf (get calibration::GetJointPositionsResponse :md5sum-) "a286ff40b196573b9ebf3999a2f8d438")
(setf (get calibration::GetJointPositionsResponse :datatype-) "calibration/GetJointPositionsResponse")
(setf (get calibration::GetJointPositionsResponse :definition-)
      "---
float64[] joint_positions

")



(provide :calibration/GetJointPositions "a286ff40b196573b9ebf3999a2f8d438")


