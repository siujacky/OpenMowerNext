# Low-level mower status message
# Reports hardware status and mower motor information

# Status constants
uint8 MOWER_STATUS_INITIALIZING = 0
uint8 MOWER_STATUS_OK = 255

builtin_interfaces/Time stamp

# System status
uint8 mower_status              # Overall mower status
bool raspberry_pi_power         # RPi power status
bool is_charging                # Currently charging
bool esc_power                  # ESC power enabled
bool rain_detected              # Rain sensor triggered
bool sound_module_available     # Sound module connected
bool sound_module_busy          # Sound module playing
bool ui_board_available         # UI board connected
bool mow_enabled                # Mowing motor enabled

# Mower ESC status
uint8 mower_esc_status          # ESC status code
float32 mower_esc_temperature   # ESC temperature [°C]
float32 mower_esc_current       # Mower motor current [A]
float32 mower_motor_temperature # Motor temperature [°C]
float32 mower_motor_rpm         # Mower motor RPM
