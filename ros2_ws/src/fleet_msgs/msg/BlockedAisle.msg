# "There is something in the way here."
# Brain class: BlockedAisle   Topic: /fleet/blocked_aisles
#
# 04_DECENTRALIZED_FLEET_PROTOCOL section 2 spells this one out, ttl and all.
# ttl matters: boxes get picked up, and a block that never faded would leave a
# phantom wall in everyone's map for ever.

string robot_id
builtin_interfaces/Time stamp
uint32 seq

int32[2] cell
float64 confidence
float64 ttl                 # seconds to believe it without seeing it again
bool cleared                # true = "I looked, and it has gone"
