# One cycle of the 3D follower (vectorfield_follower_3d), for plots and tests. Experimental in 0.x.
# Phases and status bits have the values of FieldState.
uint8 PHASE_FOLLOW=1
uint8 PHASE_FINAL_APPROACH=2
uint8 PHASE_STOPPED=4
uint32 STATUS_NON_FINITE=1
uint32 STATUS_ASSUMPTION3_CLAMPED=2
uint32 STATUS_AMBIGUOUS_CLOSEST=4
uint32 STATUS_HAIRPIN=8
uint32 STATUS_BLOCKED=256
std_msgs/Header header        # frame of all points and vectors (global_frame)
uint8 phase
uint32 status
geometry_msgs/Point position  # body position
float64 yaw                   # body yaw (rad)
float64 desired_yaw           # yaw policy target (rad; NaN if none)
geometry_msgs/Point closest_point
geometry_msgs/Vector3 tangent # traversal tangent at the closest point
float64 distance              # |D| to the curve (m)
float64 arc_length            # station of the closest point (m)
float64 distance_to_end       # NaN if closed
uint32 lap                    # laps completed
float64 progress              # progress in laps (closed curves)
float64 g                     # G(D)
float64 eta                   # eta (the eta-box of a moving curve, else 1)
geometry_msgs/Vector3 field   # Phi
geometry_msgs/Vector3 feedforward  # Phi_T
geometry_msgs/Vector3 frame_velocity  # velocity of the curve frame at the body
float64 barrier               # least CBF barrier h = D - (body_radius + cbf_margin) (m); +Inf without obstacles
bool cbf_active               # the CBF filter changed the field
geometry_msgs/Twist cmd_raw   # target before the limiter (global_frame; angular.z = yaw rate)
geometry_msgs/Twist cmd       # limited command (global_frame)
float64 limiter_scale         # limiter alpha
float64 compute_time_ms
