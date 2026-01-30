# Raw IMU data message
# Contains accelerometer, gyroscope, and magnetometer readings

uint16 dt    # Time delta since last reading [ms]

# Linear acceleration [m/s^2]
float64 ax   # X-axis acceleration
float64 ay   # Y-axis acceleration
float64 az   # Z-axis acceleration

# Angular velocity [rad/s]
float64 gx   # Angular velocity about X axis
float64 gy   # Angular velocity about Y axis
float64 gz   # Angular velocity about Z axis

# Magnetic field strength [uT]
float64 mx   # X-axis magnetic field
float64 my   # Y-axis magnetic field
float64 mz   # Z-axis magnetic field
