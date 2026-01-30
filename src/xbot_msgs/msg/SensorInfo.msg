# Sensor information message
# Describes a sensor's metadata and value ranges

# Sensor data type constants
uint8 TYPE_STRING = 1
uint8 TYPE_DOUBLE = 2

# Value description constants
uint8 VALUE_DESCRIPTION_UNKNOWN = 0
uint8 VALUE_DESCRIPTION_TEMPERATURE = 1
uint8 VALUE_DESCRIPTION_VELOCITY = 2
uint8 VALUE_DESCRIPTION_ACCELERATION = 3
uint8 VALUE_DESCRIPTION_VOLTAGE = 4
uint8 VALUE_DESCRIPTION_CURRENT = 5
uint8 VALUE_DESCRIPTION_PERCENT = 6
uint8 VALUE_DESCRIPTION_DISTANCE = 7
uint8 VALUE_DESCRIPTION_RPM = 8

# GPS flags
uint16 FLAG_GPS_RTK = 1                            # True if corrections applied
uint16 FLAG_GPS_RTK_FIXED = 2                      # True if fixed solution
uint16 FLAG_GPS_RTK_FLOAT = 4                      # True if float solution
uint16 FLAG_GPS_DEAD_RECKONING = 8                 # True if GPS is dead reckoning

# Sensor fusion flags
uint16 FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1 # True if recent absolute position
uint16 FLAG_SENSOR_FUSION_DEAD_RECKONING = 8       # True if dead reckoning

string sensor_id                 # Unique sensor identifier
string sensor_name               # Human readable name for this sensor
uint8 value_type                 # Data type of sensor data (TYPE_STRING or TYPE_DOUBLE)
uint8 value_description          # Description of value (temperature, speed, etc.)
string unit                      # Unit of the sensor data (e.g., "V" or "m/s")
bool has_min_max                 # True if min_value and max_value are valid
float64 min_value                # Absolute sensor minimum (for gauge rendering)
float64 max_value                # Absolute sensor maximum (for gauge rendering)
bool has_critical_low            # True if there's a critical lower bound
float64 lower_critical_value     # Values below this are critical
bool has_critical_high           # True if there's a critical upper bound
float64 upper_critical_value     # Values above this are critical
