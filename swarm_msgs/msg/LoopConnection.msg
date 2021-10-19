time ts_a
time ts_b
int32 id_a
int32 id_b
int32 keyframe_id_a
int32 keyframe_id_b
geometry_msgs/Pose relative_pose

geometry_msgs/Pose self_pose_a
geometry_msgs/Pose self_pose_b

geometry_msgs/Point pos_std
geometry_msgs/Point ang_std

#Details for loop
int32 pnp_inlier_num
bool is_4dof

#geometry_msgs/Point[] point_cloud_b