std_msgs/Header header

uint8 GROUNDED=0
uint8 PARTIAL_CONTACT=1
uint8 AIRBORNE=2
uint8 LANDING=3
uint8 UNKNOWN=4

uint8 state
string state_name
float32 confidence
uint8 estimated_contact_count
string[] wheel_names
float32[] wheel_contact_probability
float32 wheel_speed_mps
float32 body_speed_mps
float32 slip_ratio
float32 acceleration_norm_mps2
bool wheel_feedback_fresh
bool wheel_odom_fresh
bool body_odom_fresh
bool imu_fresh
string reason
