# "This job would cost me this much."
# Brain class: TaskBid   Topic: /fleet/task_bids
#
# 06_TASK_ALLOCATION section 3. Lowest valid cost wins. An extension to the
# protocol in 04, which stops at conflict alerts.

string robot_id
builtin_interfaces/Time stamp
uint32 seq

string task_id
float64 cost
