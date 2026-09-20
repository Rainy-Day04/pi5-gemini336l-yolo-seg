std_msgs/Header header
string task_id
string target_id
# IDLE, SELECTED, NAVIGATING, WAITING_FOR_STOP, REACQUIRING, GRASPING,
# SEQUENCE_COMPLETE, COMPLETED, FAILED, CANCELING, CANCELED, HOLD_REQUIRED.
string state
string code
string message
bool busy
# False unless a downstream sensor-backed grasp check reports success.
bool grasp_verified
