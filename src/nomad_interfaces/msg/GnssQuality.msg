# Normalized receiver quality, not injected-error ground truth.
# Same measurement epoch and antenna frame as the accompanying NavSatFix.
std_msgs/Header header
uint8 UNKNOWN=0
uint8 NO_FIX=1
uint8 SINGLE=2
uint8 DGNSS=3
uint8 RTK_FLOAT=4
uint8 RTK_FIXED=5
uint8 solution_type
uint32 SOLUTION_VALID=1
uint32 SATELLITES_VALID=2
uint32 PDOP_VALID=4
uint32 CORRECTION_AGE_VALID=8
uint32 valid_fields
uint16 satellites_used
float64 pdop
# Receiver-reported differential correction age, NOT host RTCM receive age.
# Unavailable floating values are NaN and their validity bit is clear.
float64 correction_age_s
