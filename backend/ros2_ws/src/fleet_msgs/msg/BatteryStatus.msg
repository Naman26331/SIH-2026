# "I am alive." Sent twice a second whether anything is happening or not.
# Brain class: Heartbeat   Topic: /fleet/battery (and doubles as the heartbeat)
#
# 03_ROBOT_AND_ROS2_IMPLEMENTATION section 5: if these stop arriving, the other
# robots conclude this one is gone, and release the squares it had booked.

string robot_id
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

float64 battery             # percent
string status               # MOVING, IDLE, FAILED, ...
