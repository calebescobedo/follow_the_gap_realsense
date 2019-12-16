; Auto-generated. Do not edit!


(cl:in-package ros_pololu_servo-msg)


;//! \htmlinclude pololu_trajectoryAction.msg.html

(cl:defclass <pololu_trajectoryAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type ros_pololu_servo-msg:pololu_trajectoryActionGoal
    :initform (cl:make-instance 'ros_pololu_servo-msg:pololu_trajectoryActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type ros_pololu_servo-msg:pololu_trajectoryActionResult
    :initform (cl:make-instance 'ros_pololu_servo-msg:pololu_trajectoryActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type ros_pololu_servo-msg:pololu_trajectoryActionFeedback
    :initform (cl:make-instance 'ros_pololu_servo-msg:pololu_trajectoryActionFeedback)))
)

(cl:defclass pololu_trajectoryAction (<pololu_trajectoryAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pololu_trajectoryAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pololu_trajectoryAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_pololu_servo-msg:<pololu_trajectoryAction> is deprecated: use ros_pololu_servo-msg:pololu_trajectoryAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <pololu_trajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_pololu_servo-msg:action_goal-val is deprecated.  Use ros_pololu_servo-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <pololu_trajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_pololu_servo-msg:action_result-val is deprecated.  Use ros_pololu_servo-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <pololu_trajectoryAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_pololu_servo-msg:action_feedback-val is deprecated.  Use ros_pololu_servo-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pololu_trajectoryAction>) ostream)
  "Serializes a message object of type '<pololu_trajectoryAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pololu_trajectoryAction>) istream)
  "Deserializes a message object of type '<pololu_trajectoryAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pololu_trajectoryAction>)))
  "Returns string type for a message object of type '<pololu_trajectoryAction>"
  "ros_pololu_servo/pololu_trajectoryAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pololu_trajectoryAction)))
  "Returns string type for a message object of type 'pololu_trajectoryAction"
  "ros_pololu_servo/pololu_trajectoryAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pololu_trajectoryAction>)))
  "Returns md5sum for a message object of type '<pololu_trajectoryAction>"
  "b7f2c426602baf613364470569cb9f9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pololu_trajectoryAction)))
  "Returns md5sum for a message object of type 'pololu_trajectoryAction"
  "b7f2c426602baf613364470569cb9f9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pololu_trajectoryAction>)))
  "Returns full string definition for message of type '<pololu_trajectoryAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%pololu_trajectoryActionGoal action_goal~%pololu_trajectoryActionResult action_result~%pololu_trajectoryActionFeedback action_feedback~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%pololu_trajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define goal~%trajectory_msgs/JointTrajectory joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%pololu_trajectoryResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%bool success~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%pololu_trajectoryFeedback feedback~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 percent_complete~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pololu_trajectoryAction)))
  "Returns full string definition for message of type 'pololu_trajectoryAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%pololu_trajectoryActionGoal action_goal~%pololu_trajectoryActionResult action_result~%pololu_trajectoryActionFeedback action_feedback~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%pololu_trajectoryGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define goal~%trajectory_msgs/JointTrajectory joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%pololu_trajectoryResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%bool success~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%pololu_trajectoryFeedback feedback~%~%================================================================================~%MSG: ros_pololu_servo/pololu_trajectoryFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 percent_complete~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pololu_trajectoryAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pololu_trajectoryAction>))
  "Converts a ROS message object to a list"
  (cl:list 'pololu_trajectoryAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
