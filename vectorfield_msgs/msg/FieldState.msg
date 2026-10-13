uint8 PHASE_ROTATE_TO_FIELD=0
uint8 PHASE_FOLLOW=1
uint8 PHASE_FINAL_APPROACH=2
uint8 PHASE_ROTATE_TO_HEADING=3
uint8 PHASE_STOPPED=4
uint32 STATUS_NON_FINITE=1
uint32 STATUS_ASSUMPTION3_CLAMPED=2
uint32 STATUS_AMBIGUOUS_CLOSEST=4
uint32 STATUS_HAIRPIN=8
uint32 STATUS_CURVATURE_SATURATED=16
uint32 STATUS_CONTOUR_WATCHDOG=32
uint32 STATUS_HEADING_INFEASIBLE=64
uint32 STATUS_COLLISION_AHEAD=128
uint32 STATUS_BLOCKED=256
std_msgs/Header header        # frame of all vectors
uint8 phase
uint32 status
geometry_msgs/Point control_point
geometry_msgs/Point closest_point
geometry_msgs/Vector3 tangent
float64 distance
float64 arc_length
float64 distance_to_end       # NaN if closed
float64 g
float64 h
float64 eta
geometry_msgs/Vector3 feedforward
geometry_msgs/Vector3 field
float64 obstacle_distance     # +Inf if none
float64 blend
int8 contour_direction
geometry_msgs/Twist cmd_raw
geometry_msgs/Twist cmd
float64 limiter_scale
float64 free_arc_length       # +Inf if collision-free
float64 compute_time_ms
