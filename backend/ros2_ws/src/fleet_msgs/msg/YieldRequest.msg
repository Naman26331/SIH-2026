# Decentralized PIBT request: highest-priority robot keeps its route and asks
# the blocking peer to move. Receiver inherits priority and either steps aside
# or forwards it (trail prevents loops). Only REQUEST is sent; action /
# conflict_id / winner stay on the wire for older peers and are ignored.
# Brain class: YieldRequest   Topic: /fleet/yield_requests

string robot_id             # who is asking
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string target               # who is being asked to move
int32[2] resource           # the square wanted
string reason               # PIBT, PARKED, CYCLE
string action               # REQUEST (legacy PROPOSE/YIELDING/ACK ignored)
string conflict_id          # legacy wire-compat, ignored
string winner               # legacy wire-compat, ignored
int32 priority              # PIBT priority inherited through local chain
string root                 # peer that started the chain
string[] trail              # visited peers; prevents request loops
