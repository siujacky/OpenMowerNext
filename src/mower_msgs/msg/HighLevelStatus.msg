# High-level mower status message
# Reports the overall state of the mower including mode, area, and battery

# High-level state constants
uint8 HIGH_LEVEL_STATE_NULL = 0
uint8 HIGH_LEVEL_STATE_IDLE = 1
uint8 HIGH_LEVEL_STATE_AUTONOMOUS = 2
uint8 HIGH_LEVEL_STATE_RECORDING = 3

# Substate constants
uint8 SUBSTATE_1 = 0
uint8 SUBSTATE_2 = 1
uint8 SUBSTATE_3 = 2
uint8 SUBSTATE_4 = 3
uint8 SUBSTATE_SHIFT = 6

uint8 state                  # Current high-level state
string state_name            # Human-readable state name
string sub_state_name        # Human-readable sub-state name

int16 current_area           # Current mowing area index
int16 current_path           # Current path index
int16 current_path_index     # Current position in path

float32 gps_quality_percent  # GPS signal quality [0-100%]
float32 battery_percent      # Battery charge level [0-100%]

bool is_charging             # Currently charging
bool emergency               # Emergency state active
