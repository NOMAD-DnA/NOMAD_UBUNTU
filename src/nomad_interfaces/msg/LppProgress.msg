# Optional LPP -> GPP coordination, reliable/volatile depth 1.
# NOT motion authorization. Existing PlannedMotion and command safety checks apply.
uint8 TRACKING=0
uint8 AVOIDING=1
uint8 BLOCKED=2
std_msgs/Header header
# Exact header.stamp of the /nomad/global_path used by this solve.
builtin_interfaces/Time reference_stamp
uint8 state
bool request_replan
# Forward rejoin point in the referenced Path. Fixed during one avoidance episode.
uint32 rejoin_index
# Exact same checked bundle also published on /nomad/planning/local_motion.
nomad_interfaces/PlannedMotion motion
