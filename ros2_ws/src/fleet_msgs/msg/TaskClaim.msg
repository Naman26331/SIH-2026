# "I am taking this job" / "I have picked it up" / "delivered" / "giving it back".
# Brain class: TaskClaim   Topic: /fleet/task_claims

string robot_id
builtin_interfaces/Time stamp
uint32 seq

string task_id
string action               # CLAIM, PICKED, DONE or RELEASE
float64 cost                # the winning bid, so others can check it
