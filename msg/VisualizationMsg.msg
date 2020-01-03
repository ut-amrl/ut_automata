std_msgs/Header header
# Namespace
string ns

# Provided by particle filter
Pose2Df[] particles

# Provided by navigation
# Assumes that the last one in the list is the best option.
PathVisualization[] path_options

# Generic visualization types.
ColoredPoint2D[] points
ColoredLine2D[] lines
ColoredArc2D[] arcs
