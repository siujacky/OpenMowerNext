# Absolute pose message
# Describes a drift-free absolute position measured by some sensor

# Source type constants
uint8 SOURCE_GPS = 1
uint8 SOURCE_LIGHTHOUSE = 2
uint8 SOURCE_SENSOR_FUSION = 100

# GPS flags
uint16 FLAG_GPS_RTK = 1                            # Corrections are applied
uint16 FLAG_GPS_RTK_FIXED = 2                      # Fixed solution
uint16 FLAG_GPS_RTK_FLOAT = 4                      # Float solution
uint16 FLAG_GPS_DEAD_RECKONING = 8                 # GPS is doing dead reckoning with internal IMU

# Sensor fusion flags
uint16 FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1 # Recent absolute position measurement
uint16 FLAG_SENSOR_FUSION_DEAD_RECKONING = 8       # Dead reckoning mode

std_msgs/Header header

uint32 sensor_stamp          # Timestamp sent by the sensor (0 for sensor fusion)
uint32 received_stamp        # Timestamp taken on receive (0 for sensor fusion)

uint8 source                 # Source type (GPS, lighthouse, or sensor fusion)
uint16 flags                 # Source-specific flags

uint8 orientation_valid      # True if sensor can measure orientation
uint8 motion_vector_valid    # True if speed vector is valid

float32 position_accuracy    # Estimated accuracy in meters
float32 orientation_accuracy # Estimated orientation accuracy in radians

geometry_msgs/PoseWithCovariance pose    # The measured pose
geometry_msgs/Vector3 motion_vector      # Motion vector in m/s
float64 vehicle_heading                  # Yaw for the orientation
float64 motion_heading                   # Yaw for the motion vector
