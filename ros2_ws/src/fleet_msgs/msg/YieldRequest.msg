# "Excuse me, you are in my way -- please move."
# Brain class: YieldRequest   Topic: /fleet/yield_requests
#
# Sent to a robot parked on a square somebody needs, or to the one chosen to
# give way and break a circular wait. An extension to the protocol in 04.

string robot_id             # who is asking
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string target               # who is being asked to move
int32[2] resource           # the square wanted
string reason               # PARKED, CYCLE or BOXED_IN
