
(cl:in-package :asdf)

(defsystem "dfDrone-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DFDMessage" :depends-on ("_package_DFDMessage"))
    (:file "_package_DFDMessage" :depends-on ("_package"))
  ))