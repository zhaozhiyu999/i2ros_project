
(cl:in-package :asdf)

(defsystem "simulation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "VehicleControl" :depends-on ("_package_VehicleControl"))
    (:file "_package_VehicleControl" :depends-on ("_package"))
  ))