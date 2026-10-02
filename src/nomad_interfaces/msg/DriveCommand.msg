# One authoritative planning command. Header stamp uses ROS time.
# frame_id is base_link; speed is longitudinal rear-axle m/s.
std_msgs/Header header
# Positive forward, negative reverse.
float64 target_speed
# Virtual centre front wheel angle, rad. Positive left, including reverse.
float64 target_steering_angle
# Lease from header.stamp, capped by the receiver's configured maximum.
builtin_interfaces/Duration valid_for
# Overrides numeric targets: stop. A stop must never resume cached motion.
bool stop_requested
