
(cl:in-package :asdf)

(defsystem "ORB_VIO-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "viorb_msg" :depends-on ("_package_viorb_msg"))
    (:file "_package_viorb_msg" :depends-on ("_package"))
  ))