# "I am booking this square for this stretch of time."
# Brain class: PathReservation   Topic: /fleet/reservations
#
# Everyone who hears it writes it into their own copy of the table. Nobody
# GRANTS it -- ownership is worked out separately by a rule every robot applies
# identically, so the answer does not depend on what order messages arrive in.

string robot_id
builtin_interfaces/Time stamp
uint32 seq

string action               # CLAIM or RELEASE
string kind                 # NODE or EDGE
int32[] cells               # flattened x,y pairs: one square, or an edge's two ends
float64 start               # absolute times, not "seconds from now"
float64 end
int32 priority
