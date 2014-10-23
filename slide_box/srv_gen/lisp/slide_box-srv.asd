
(cl:in-package :asdf)

(defsystem "slide_box-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "robot_actuate_object" :depends-on ("_package_robot_actuate_object"))
    (:file "_package_robot_actuate_object" :depends-on ("_package"))
  ))