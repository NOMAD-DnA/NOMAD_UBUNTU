# Assembly timestamp in ROS time, frame_id=base_link.
# Valid flags require fresh finite measurements, not echoed command targets.
std_msgs/Header header
float64 speed
float64 steering_angle
bool speed_valid
bool steering_valid
bool ready
bool fault
string detail
