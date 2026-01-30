# XESC Motor Controller State Message
# Ported from open_mower_ros to ROS2

# Fault codes (bitmask)
uint32 XESC_FAULT_UNINITIALIZED=1
uint32 XESC_FAULT_WATCHDOG=2
uint32 XESC_FAULT_UNDERVOLTAGE=4
uint32 XESC_FAULT_OVERVOLTAGE=8
uint32 XESC_FAULT_OVERCURRENT=16
uint32 XESC_FAULT_OVERTEMP_MOTOR=32
uint32 XESC_FAULT_OVERTEMP_PCB=64
uint32 XESC_FAULT_INVALID_HALL=128
uint32 XESC_FAULT_INTERNAL_ERROR=256
uint32 XESC_FAULT_OPEN_LOAD=512

# Connection states
uint8 XESC_CONNECTION_STATE_DISCONNECTED=0
uint8 XESC_CONNECTION_STATE_WAITING_FOR_FW=1
uint8 XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW=2
uint8 XESC_CONNECTION_STATE_CONNECTED=3

# State data
uint8   connection_state     # Current connection state
uint8   fw_major             # Firmware major version
uint8   fw_minor             # Firmware minor version
float64 voltage_input        # Input voltage (volt)
float64 temperature_pcb      # Temperature of printed circuit board (degrees Celsius)
float64 temperature_motor    # Temperature of connected motor (degrees Celsius)
float64 current_input        # Input current (ampere)
float64 duty_cycle           # Duty cycle (0 to 1)
uint8   direction            # Current direction
uint32  tacho                # Wheel ticks (relative)
uint32  tacho_absolute       # Wheel ticks (absolute)
int16   rpm                  # Revolutions per minute (of the axis/shaft)
uint32  fault_code           # Fault code bitmask
