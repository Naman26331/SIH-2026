# A job: collect from here, deliver to there.
# Brain class: TaskAnnounce   Topic: /fleet/tasks
#
# Fields follow the data model in 02_TECHNICAL_ARCHITECTURE section 5.

string robot_id             # who announced it (the order system, or a robot re-announcing)
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string task_id
int32[2] pickup
int32[2] dropoff
string product
string shelf                # Phase 11: the rack it comes off, e.g. "A14"
int32 quantity              # how many
int32 priority
bool flexible               # true = any packing station will do
