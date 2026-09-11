# Where a robot is ABOUT to be. The important one.
# Brain class: IntentUpdate   Topic: /fleet/robot_intents
#
# 04_DECENTRALIZED_FLEET_PROTOCOL section 3:
#   Position-only:  "R1 is here."
#   Intent-aware:   "R1 is here. R1 wants to go there. R1 expects to occupy
#                    these nodes. R1 expects to arrive at this time."
#
# Sent only when the plan CHANGES, not on a timer. Repeating an unchanged plan
# twenty times a second would flood real warehouse Wi-Fi.

string robot_id
builtin_interfaces/Time stamp
uint32 seq
string tag                  # Phase 23: HMAC signature -- see security.py

float64 x
float64 y
float64 velocity

bool has_destination        # false when the robot has nowhere to go
int32[2] destination

int32[] planned_nodes       # flattened x,y pairs: [x0,y0, x1,y1, ...]
float64[] node_etas         # seconds from stamp, one per planned node
bool has_eta_destination
float64 eta_destination

int32 priority              # tenths of a point; see priority.py
string status               # MOVING, WAITING, REROUTING, YIELDING, ...
