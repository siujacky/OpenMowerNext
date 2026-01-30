# Robot state message
# High-level state information for display to users

float32 battery_percentage      # Battery charge percentage
bool emergency                  # Emergency state active
bool is_charging                # Currently charging
bool rain_detected              # Rain sensor triggered
float32 gps_percentage          # GPS quality percentage
float32 current_action_progress # Progress of current action [0-100%]
string current_state            # Current state (e.g., "Cleaning")
string current_sub_state        # Current sub-state (e.g., "doing the dishes")
int16 current_area              # Current mowing area index
int16 current_path              # Current path index
int16 current_path_index        # Current position in path
xbot_msgs/AbsolutePose robot_pose   # Current robot pose
