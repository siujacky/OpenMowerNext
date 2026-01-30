# PID controller state message
# Reports the state of the follow-the-carrot planner PID controllers

builtin_interfaces/Time stamp

# P gain setpoints
float32 kp_lon_set    # Longitudinal P gain
float32 kp_lat_set    # Lateral P gain
float32 kp_ang_set    # Angular P gain

# I gain setpoints
float32 ki_lon_set    # Longitudinal I gain
float32 ki_lat_set    # Lateral I gain
float32 ki_ang_set    # Angular I gain

# D gain setpoints
float32 kd_lon_set    # Longitudinal D gain
float32 kd_lat_set    # Lateral D gain
float32 kd_ang_set    # Angular D gain

# Current errors
float32 lon_err       # Longitudinal error
float32 lat_err       # Lateral error
float32 ang_err       # Angular error

# Output speeds
float32 ang_speed     # Angular speed command
float32 lin_speed     # Linear speed command
