
(cl:in-package :asdf)

(defsystem "thesis_realsense-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "GripperData" :depends-on ("_package_GripperData"))
    (:file "_package_GripperData" :depends-on ("_package"))
  ))