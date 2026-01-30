# Checkpoint message for resuming mowing operations
# Stores state needed to resume mowing after interruption

int32 currentMowingPath              # Current path index
int32 currentMowingArea              # Current area index
int32 currentMowingPathIndex         # Position within current path
string currentMowingPlanDigest       # Hash of the mowing plan
float32 currentMowingAngleIncrementSum    # Accumulated angle changes
