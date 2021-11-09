Header header
int64 id
int32 self_drone_id
int32 remote_drone_id
geometry_msgs/Point dpos
geometry_msgs/Point dpos_std

float64 dyaw
float64 dyaw_cov
bool enable_scale
bool is_yaw_valid
float64 probaility
float64 inv_dep

geometry_msgs/Pose local_pose_self #Here is camera pose, not IMU pose.
geometry_msgs/Pose local_pose_remote

geometry_msgs/Pose camera_extrinsic