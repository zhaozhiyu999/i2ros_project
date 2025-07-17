;; Auto-generated. Do not edit!


(when (boundp 'simulation::VehicleControl)
  (if (not (find-package "SIMULATION"))
    (make-package "SIMULATION"))
  (shadow 'VehicleControl (find-package "SIMULATION")))
(unless (find-package "SIMULATION::VEHICLECONTROL")
  (make-package "SIMULATION::VEHICLECONTROL"))

(in-package "ROS")
;;//! \htmlinclude VehicleControl.msg.html


(defclass simulation::VehicleControl
  :super ros::object
  :slots (_Throttle _Steering _Brake _Reserved ))

(defmethod simulation::VehicleControl
  (:init
   (&key
    ((:Throttle __Throttle) 0.0)
    ((:Steering __Steering) 0.0)
    ((:Brake __Brake) 0.0)
    ((:Reserved __Reserved) 0.0)
    )
   (send-super :init)
   (setq _Throttle (float __Throttle))
   (setq _Steering (float __Steering))
   (setq _Brake (float __Brake))
   (setq _Reserved (float __Reserved))
   self)
  (:Throttle
   (&optional __Throttle)
   (if __Throttle (setq _Throttle __Throttle)) _Throttle)
  (:Steering
   (&optional __Steering)
   (if __Steering (setq _Steering __Steering)) _Steering)
  (:Brake
   (&optional __Brake)
   (if __Brake (setq _Brake __Brake)) _Brake)
  (:Reserved
   (&optional __Reserved)
   (if __Reserved (setq _Reserved __Reserved)) _Reserved)
  (:serialization-length
   ()
   (+
    ;; float32 _Throttle
    4
    ;; float32 _Steering
    4
    ;; float32 _Brake
    4
    ;; float32 _Reserved
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Throttle
       (sys::poke _Throttle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Steering
       (sys::poke _Steering (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Brake
       (sys::poke _Brake (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Reserved
       (sys::poke _Reserved (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Throttle
     (setq _Throttle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Steering
     (setq _Steering (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Brake
     (setq _Brake (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Reserved
     (setq _Reserved (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get simulation::VehicleControl :md5sum-) "0d66040a38beac1d32107b24d8aabeae")
(setf (get simulation::VehicleControl :datatype-) "simulation/VehicleControl")
(setf (get simulation::VehicleControl :definition-)
      "float32 Throttle
float32 Steering
float32 Brake
float32 Reserved
")



(provide :simulation/VehicleControl "0d66040a38beac1d32107b24d8aabeae")


