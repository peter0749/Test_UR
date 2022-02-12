
(cl:in-package :asdf)

(defsystem "calibration-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetJointPositions" :depends-on ("_package_GetJointPositions"))
    (:file "_package_GetJointPositions" :depends-on ("_package"))
    (:file "SetJointPositions" :depends-on ("_package_SetJointPositions"))
    (:file "_package_SetJointPositions" :depends-on ("_package"))
  ))