# "You and I are going to want the same square at the same time."
# Brain class: ConflictAlert   Topic: /fleet/conflicts
#
# 04_DECENTRALIZED_FLEET_PROTOCOL section 2.

string robot_id             # who raised it
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string robot_a
string robot_b
int32[2] resource           # the contested square
string kind                 # SAME_SQUARE, HEAD_ON or JUNCTION
float64 estimated_time      # seconds from now until the clash
