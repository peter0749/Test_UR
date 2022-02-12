;; Auto-generated. Do not edit!


(when (boundp 'calibration::SetJointPositions)
  (if (not (find-package "CALIBRATION"))
    (make-package "CALIBRATION"))
  (shadow 'SetJointPositions (find-package "CALIBRATION")))
(unless (find-package "CALIBRATION::SETJOINTPOSITIONS")
  (make-package "CALIBRATION::SETJOINTPOSITIONS"))
(unless (find-package "CALIBRATION::SETJOINTPOSITIONSREQUEST")
  (make-package "CALIBRATION::SETJOINTPOSITIONSREQUEST"))
(unless (find-package "CALIBRATION::SETJOINTPOSITIONSRESPONSE")
  (make-package "CALIBRATION::SETJOINTPOSITIONSRESPONSE"))

(in-package "ROS")





(defclass calibration::SetJointPositionsRequest
  :super ros::object
  :slots (_joint_positions ))

(defmethod calibration::SetJointPositionsRequest
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

(defclass calibration::SetJointPositionsResponse
  :super ros::object
  :slots (_result ))

(defmethod calibration::SetJointPositionsResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass calibration::SetJointPositions
  :super ros::object
  :slots ())

(setf (get calibration::SetJointPositions :md5sum-) "702c3dade1da9e295e178fcc32cce64f")
(setf (get calibration::SetJointPositions :datatype-) "calibration/SetJointPositions")
(setf (get calibration::SetJointPositions :request) calibration::SetJointPositionsRequest)
(setf (get calibration::SetJointPositions :response) calibration::SetJointPositionsResponse)

(defmethod calibration::SetJointPositionsRequest
  (:response () (instance calibration::SetJointPositionsResponse :init)))

(setf (get calibration::SetJointPositionsRequest :md5sum-) "702c3dade1da9e295e178fcc32cce64f")
(setf (get calibration::SetJointPositionsRequest :datatype-) "calibration/SetJointPositionsRequest")
(setf (get calibration::SetJointPositionsRequest :definition-)
      "float64[] joint_positions
---
bool result

")

(setf (get calibration::SetJointPositionsResponse :md5sum-) "702c3dade1da9e295e178fcc32cce64f")
(setf (get calibration::SetJointPositionsResponse :datatype-) "calibration/SetJointPositionsResponse")
(setf (get calibration::SetJointPositionsResponse :definition-)
      "float64[] joint_positions
---
bool result

")



(provide :calibration/SetJointPositions "702c3dade1da9e295e178fcc32cce64f")


