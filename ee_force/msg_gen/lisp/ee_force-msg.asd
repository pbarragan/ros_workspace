
(cl:in-package :asdf)

(defsystem "ee_force-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "eeForceMsg" :depends-on ("_package_eeForceMsg"))
    (:file "_package_eeForceMsg" :depends-on ("_package"))
  ))