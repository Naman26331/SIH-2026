# Signed operator command from dashboard gateway to one robot.
string robot_id             # sender; always OPERATOR
builtin_interfaces/Time stamp
uint32 seq
string tag

string target_robot
int32[] target              # grid x,y

