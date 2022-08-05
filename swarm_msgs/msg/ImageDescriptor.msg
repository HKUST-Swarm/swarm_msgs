Header header
int32 drone_id
int32 camera_index
int32 camera_id
int32 frame_id

float32[] landmark_descriptor
float32[] landmark_scores

geometry_msgs/Pose pose_drone
geometry_msgs/Pose camera_extrinsic

float32[] image_desc
int32 image_width
int32 image_height
uint8[] image

Landmark[] landmarks

bool prevent_adding_db