# Planning-internal atomic bundle: checked geometry AND its generating command.
# path.header is the generation timestamp/frame; points are base_link poses.
nav_msgs/Path path
float64 target_speed
float64 target_steering_angle
# Recovery owns execution while active, even if its path is empty (hold).
bool active
