# A curve in R^3 for FollowCurve3 (vectorfield_core::d3::CurveSpec3). Experimental in 0.x.
uint8 POLYLINE=0
uint8 EXPRESSION=1
uint8 BUILTIN=2
std_msgs/Header header        # frame_id = curve frame ("" = global_frame); stamp = t0 of r(s,t) (zero = goal acceptance)
uint8 type
bool closed                   # POLYLINE and EXPRESSION; built-ins decide (helix is open)
geometry_msgs/Point[] points  # POLYLINE vertices (x, y, z)
string x_expression           # EXPRESSION: muParser in s, t, parameter_names
string y_expression
string z_expression
float64 s_min                 # EXPRESSION: required domain; BUILTIN: both 0 = natural domain; POLYLINE: ignored
float64 s_max
string builtin_shape          # helix|circle|ellipse|viviani|lemniscate3d|torus_knot
string[] parameter_names
float64[] parameter_values
geometry_msgs/Pose pose       # BUILTIN: placement in the curve frame (all-zero quaternion = identity); identity for the other types
float64 sample_spacing        # m, <=0: node default
