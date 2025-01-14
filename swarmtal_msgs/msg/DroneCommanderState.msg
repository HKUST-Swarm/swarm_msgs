uint32 LANDING_MODE_XYVEL = 0 #0 vo landing
uint32 LANDING_MODE_ATT = 1 #1 zero attitude landing

DroneCommanderCoreState core_state

float64 takeoff_target_height
float64 takeoff_velocity
uint32 landing_mode
float64 landing_velocity

geometry_msgs/Point vel

DronePosCtrlCmd ctrl_cmd
sensor_msgs/Imu imu_data
