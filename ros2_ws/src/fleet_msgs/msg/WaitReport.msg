# "I am stuck, and this is who I am stuck behind."
# Brain class: WaitReport   Topic: /fleet/wait_reports
#
# Everyone collects these and builds the little map of who is waiting for whom.
# 05_PATH_PLANNING section 8 calls it a wait-for graph; a loop in it is a
# deadlock. An extension to the protocol in 04.

string robot_id
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

bool is_blocked             # false = not stuck at all
string blocked_by
float64 waiting             # seconds stuck
int32 priority
bool is_moving              # false = parked, so it will never clear on its own
