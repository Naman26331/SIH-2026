# "There is something in the way here."
# Brain class: BlockedAisle   Topic: /fleet/blocked_aisles
#
# ttl remains for wire compatibility. Zero means the obstacle stays blocked
# until a later message explicitly sets cleared=true.

string robot_id
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

int32[2] cell
float64 confidence
float64 ttl                 # zero = persist until explicitly cleared
bool cleared                # true = "I looked, and it has gone"
