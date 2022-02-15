time ts_a
time ts_b
uint64 drone_id_a
uint64 drone_id_b
uint64 keyframe_id_a
uint64 keyframe_id_b
geometry_msgs/Pose relative_pose

geometry_msgs/Pose self_pose_a
geometry_msgs/Pose self_pose_b

geometry_msgs/Point pos_cov
geometry_msgs/Point ang_cov

#Details for loop
int32 pnp_inlier_num
bool is_4dof
uint64 id

#geometry_msgs/Point[] point_cloud_b