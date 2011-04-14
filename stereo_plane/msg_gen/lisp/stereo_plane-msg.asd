
(cl:in-package :asdf)

(defsystem "stereo_plane-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Plane" :depends-on ("_package_Plane"))
    (:file "_package_Plane" :depends-on ("_package"))
  ))