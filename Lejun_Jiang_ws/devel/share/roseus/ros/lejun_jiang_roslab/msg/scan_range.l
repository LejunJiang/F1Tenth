;; Auto-generated. Do not edit!


(when (boundp 'lejun_jiang_roslab::scan_range)
  (if (not (find-package "LEJUN_JIANG_ROSLAB"))
    (make-package "LEJUN_JIANG_ROSLAB"))
  (shadow 'scan_range (find-package "LEJUN_JIANG_ROSLAB")))
(unless (find-package "LEJUN_JIANG_ROSLAB::SCAN_RANGE")
  (make-package "LEJUN_JIANG_ROSLAB::SCAN_RANGE"))

(in-package "ROS")
;;//! \htmlinclude scan_range.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lejun_jiang_roslab::scan_range
  :super ros::object
  :slots (_header _closest_point _farthest_point ))

(defmethod lejun_jiang_roslab::scan_range
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:closest_point __closest_point) 0.0)
    ((:farthest_point __farthest_point) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _closest_point (float __closest_point))
   (setq _farthest_point (float __farthest_point))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:closest_point
   (&optional __closest_point)
   (if __closest_point (setq _closest_point __closest_point)) _closest_point)
  (:farthest_point
   (&optional __farthest_point)
   (if __farthest_point (setq _farthest_point __farthest_point)) _farthest_point)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _closest_point
    8
    ;; float64 _farthest_point
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _closest_point
       (sys::poke _closest_point (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _farthest_point
       (sys::poke _farthest_point (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _closest_point
     (setq _closest_point (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _farthest_point
     (setq _farthest_point (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get lejun_jiang_roslab::scan_range :md5sum-) "492e98f4ad5413e40d76ac2fceece9f6")
(setf (get lejun_jiang_roslab::scan_range :datatype-) "lejun_jiang_roslab/scan_range")
(setf (get lejun_jiang_roslab::scan_range :definition-)
      "Header header
float64 closest_point
float64 farthest_point
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :lejun_jiang_roslab/scan_range "492e98f4ad5413e40d76ac2fceece9f6")


