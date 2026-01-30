# Map area message
# Represents a mowing area with optional obstacles

string name                          # Name/identifier for this area
geometry_msgs/Polygon area           # Outer boundary of the area
geometry_msgs/Polygon[] obstacles    # Obstacles within the area
