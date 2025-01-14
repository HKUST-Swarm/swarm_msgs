uint8 CTRL_INPUT_NONE=0
uint8 CTRL_INPUT_RC=1
uint8 CTRL_INPUT_ONBOARD=2

uint8 FLIGHT_STATUS_IDLE=0 # Motor Stopped, on ground
uint8 FLIGHT_STATUS_ARMED=1 # Armed, on ground
uint8 FLIGHT_STATUS_IN_AIR=2 #Armed Inair
uint8 FLIGHT_STATUS_CRASHED=3 # Crashed, Armed


uint8 CTRL_AUTH_RC = 0
uint8 CTRL_AUTH_APP = 1
uint8 CTRL_AUTH_THIS = 2

uint8 CTRL_MODE_IDLE=0
uint8 CTRL_MODE_TAKEOFF=1
uint8 CTRL_MODE_LANDING=2
uint8 CTRL_MODE_HOVER=3
uint8 CTRL_MODE_POSVEL=4
uint8 CTRL_MODE_ATT=5
uint8 CTRL_MODE_MISSION=6
uint8 CTRL_MODE_ALT=7

std_msgs/Header header
uint32 drone_id

uint8 ctrl_input_state
uint8 flight_status
uint8 control_auth
uint8 commander_ctrl_mode

bool is_armed
bool rc_valid
bool onboard_cmd_valid
bool vo_valid
bool mavros_valid

float32 vo_latency

float32 bat_vol
float32 bat_remain

geometry_msgs/Point32 pos
float32 yaw
