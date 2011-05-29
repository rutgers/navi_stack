
(cl:in-package :asdf)

(defsystem "camera_pose_calibration-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :calibration_msgs-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CalibrationEstimate" :depends-on ("_package_CalibrationEstimate"))
    (:file "_package_CalibrationEstimate" :depends-on ("_package"))
    (:file "CameraCalibration" :depends-on ("_package_CameraCalibration"))
    (:file "_package_CameraCalibration" :depends-on ("_package"))
    (:file "CameraMeasurement" :depends-on ("_package_CameraMeasurement"))
    (:file "_package_CameraMeasurement" :depends-on ("_package"))
    (:file "CameraPose" :depends-on ("_package_CameraPose"))
    (:file "_package_CameraPose" :depends-on ("_package"))
    (:file "RobotMeasurement" :depends-on ("_package_RobotMeasurement"))
    (:file "_package_RobotMeasurement" :depends-on ("_package"))
  ))