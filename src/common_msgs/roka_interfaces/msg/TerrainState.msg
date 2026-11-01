std_msgs/Header header

uint8 NORMAL=0
uint8 UPHILL=1
uint8 DOWNHILL=2
uint8 CROSS_SLOPE=3
uint8 SLIP=4
uint8 FAULT=5

uint8 state
string state_name
float32 speed_scale
float32 risk
float32 longitudinal_slope_deg
float32 lateral_slope_deg
float32 roughness_m
float32 step_height_m
bool terrain_fresh
bool imu_fresh
bool odom_fresh
string reason
