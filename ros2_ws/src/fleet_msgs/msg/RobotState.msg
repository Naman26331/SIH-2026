# Where a robot is right now.
# Brain class: PoseUpdate   Topic: /fleet/robot_states
#
# Sent about ten times a second, because position changes fast.

string robot_id
builtin_interfaces/Time stamp
uint32 seq                  # counts up; a gap proves a message was lost

float64 x                   # grid coordinates, not metres. See translate.py.
float64 y
int32[2] cell               # the square it is on or has just left
float64 velocity            # squares per second, 0 when actually stopped
string heading              # N, E, S or W
