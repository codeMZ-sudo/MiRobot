
(cl:in-package :asdf)

(defsystem "gazebo_omni_plugins-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gazebo_omni_plugins_pid" :depends-on ("_package_gazebo_omni_plugins_pid"))
    (:file "_package_gazebo_omni_plugins_pid" :depends-on ("_package"))
    (:file "gazebo_omni_plugins_vel" :depends-on ("_package_gazebo_omni_plugins_vel"))
    (:file "_package_gazebo_omni_plugins_vel" :depends-on ("_package"))
  ))