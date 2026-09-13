# Decentralized PIBT request: highest-priority robot keeps its route and asks
# the blocking peer to move. Receiver inherits priority and either steps aside
# or forwards it (trail prevents loops). ACCEPT unwinds a successful chain;
# REJECT makes the parent backtrack to another local candidate.
# Brain class: YieldRequest   Topic: /fleet/yield_requests

string robot_id             # who is asking
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

string target               # who is being asked to move
int32[2] resource           # the square wanted
string reason               # PIBT, PARKED, CYCLE
string action               # REQUEST, ACCEPT or REJECT
string conflict_id          # stable id for one branch of the request chain
string winner               # legacy wire-compat, ignored
int32 priority              # PIBT priority inherited through local chain
string root                 # peer that started the chain
string[] trail              # visited peers; prevents request loops
