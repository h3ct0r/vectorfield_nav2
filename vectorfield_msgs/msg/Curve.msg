uint8 POLYLINE=0
uint8 EXPRESSION=1
uint8 BUILTIN=2
std_msgs/Header header        # frame_id = curve frame; stamp = t0 of r(s,t) (zero = goal acceptance). Only frame/time reference of a goal.
uint8 type
bool closed                   # ignored for BUILTIN (all built-ins are closed)
geometry_msgs/Point[] points  # POLYLINE (float64; z ignored)
string x_expression           # EXPRESSION: muParser in s, t, parameter_names
string y_expression
float64 s_min                 # EXPRESSION: required domain; BUILTIN: both 0 = natural domain; POLYLINE: ignored
float64 s_max
string builtin_shape          # ellipse|lemniscate_bernoulli|lemniscate_gerono|superellipse|rounded_rectangle|lissajous|regular_polygon
string[] parameter_names
float64[] parameter_values
float64 sample_spacing        # m, <=0: node default
float64 max_corner_deviation  # m, <=0: no corner smoothing
