std_msgs/Header header
# Header frame_id must be "base_link"

uint8 BEARING_ONLY=0
uint8 BEARING_RANGE=1
uint8 measurement_type

string class_name
float32 confidence

# Unit bearing vector expressed in base_link frame
geometry_msgs/Vector3 bearing_unit

# Anisotropic bearing uncertainty (radians) derived from bbox size and camera intrinsics
float32 sigma_azimuth_rad
float32 sigma_elevation_rad

# Valid only when measurement_type == BEARING_RANGE
float32 range_m
float32 sigma_range_m
