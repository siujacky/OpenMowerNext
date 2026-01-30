# Emergency state message
# Reports current and latched emergency status

builtin_interfaces/Time stamp

bool active_emergency    # Currently in emergency state
bool latched_emergency   # Emergency has been triggered (needs manual reset)
string reason            # Description of emergency cause
