# "I am taking this job" / "I have picked it up" / "delivered" / "giving it back".
# Brain class: TaskClaim   Topic: /fleet/task_claims

string robot_id
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string task_id
string action               # CLAIM, PICKED, DONE or RELEASE
string sender                # Phase 23: who is ACTUALLY speaking, if not robot_id
                             # -- see TaskClaim.sender in messages.py
float64 cost                 # the winning bid, so others can check it
