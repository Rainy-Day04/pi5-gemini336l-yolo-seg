# Contract v1: SI units. target_id is a UUID assigned on selection, not a tracker ID.
string target_id
# Frame-local YOLO instance ID; meaningful ONLY with the observation stamp below.
int32 source_instance_id
int32 class_id
string class_name
float32 confidence
# Object position in configured fixed world frame (map by default).
# Header stamp is the source image stamp, NOT the time this message was sent.
geometry_msgs/PointStamped world_point
# Same observation transformed into robot base frame at that exact stamp.
# x forward, y left, z up; base_point is a snapshot, NOT a navigation destination.
geometry_msgs/PointStamped base_point
float32 planar_distance_m
float32 bearing_rad
