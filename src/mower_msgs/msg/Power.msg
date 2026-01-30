# Power system status message
# Reports battery and charging information

builtin_interfaces/Time stamp

float32 v_charge         # Charger voltage [V]
float32 v_battery        # Battery voltage [V]
float32 charge_current   # Charging current [A]
bool charger_enabled     # Whether charger is enabled
string charger_status    # Human-readable charger status
