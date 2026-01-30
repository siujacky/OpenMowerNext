# Wheel tick encoder message
# Reports encoder counts from up to four wheels

# Valid wheel bitfield values
uint8 WHEEL_VALID_FL = 1   # Front left valid
uint8 WHEEL_VALID_FR = 2   # Front right valid
uint8 WHEEL_VALID_RL = 4   # Rear left valid
uint8 WHEEL_VALID_RR = 8   # Rear right valid

builtin_interfaces/Time stamp

uint32 wheel_tick_factor     # Factor to convert wheel ticks to meters
uint8 valid_wheels           # Bitfield showing which ticks are valid

# Front left wheel
uint8 wheel_direction_fl     # Direction of front left wheel
uint32 wheel_ticks_fl        # Tick count front left

# Front right wheel
uint8 wheel_direction_fr     # Direction of front right wheel
uint32 wheel_ticks_fr        # Tick count front right

# Rear left wheel
uint8 wheel_direction_rl     # Direction of rear left wheel
uint32 wheel_ticks_rl        # Tick count rear left

# Rear right wheel
uint8 wheel_direction_rr     # Direction of rear right wheel
uint32 wheel_ticks_rr        # Tick count rear right
