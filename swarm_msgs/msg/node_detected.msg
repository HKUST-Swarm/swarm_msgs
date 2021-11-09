Header header
int64 id
int32 self_drone_id
int32 remote_drone_id
geometry_msgs/PoseWithCovariance relative_pose #If 2d detect, gives x=1
bool is_2d_detect
bool is_yaw_valid
float64 probaility
geometry_msgs/Pose local_pose_self
geometry_msgs/Pose local_pose_remote
bool dof_4