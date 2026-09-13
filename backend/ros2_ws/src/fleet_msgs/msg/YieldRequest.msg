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
string action               # REQUEST, PROPOSE, YIELDING or ACK
string conflict_id          # stable id shared by the robot pair
string winner               # robot that keeps its current route
int32 priority              # PIBT priority inherited through local chain
string root                 # peer that started the chain
string[] trail              # visited peers; prevents request loops
