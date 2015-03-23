# Pose for which the information should be calculated. TODO/NOT IMPLEMENTED YET: If more than one then predictions into
# the future will be attempted by simulating measurements for all poses up to the last and then
# returning the predicted informations for the last pose (given that the other poses were visited before)
geometry_msgs/Pose[] poses

# Object hypothesis that should be used to retrieve the information (e.g. "SimpleSurface" takes
# the data as is)
string hypothesis_method

# How many rays are cast per pixel on the image's x-axis to obtain the information [rays/px]
float64 ray_resolution_x

# How many rays are cast per pixel on the image's y-axis to obtain the information [rays/px]
float64 ray_resolution_y

# Minimal range for rays (if the camera position happens to be at an occupied voxel, this enforces raycasting), origin of ray will be offset by this length once to attempt a recast
float64 min_ray_depth

# Maximal depth a ray may reach, if not set, currently a default might be enforced (10m)
float64 max_ray_depth

# If the occupancy likelihood of the end point of a ray is lower than this threshold, the ray is
# continued until either an endpoint is found that has a higher occupancy likelihood or the maximal
# ray depth is reached
float64 occupied_passthrough_threshold

# name(s) of the metric to use to obtain the information. If more than one is specified, each is
# calculated and returned independently
string[] metric_names

# x-ray resolution used for predictions (see explanations for poses), if zero(not specified), then the same resolution as for information retrieval is used
float64 forecast_ray_resolution_x

# y-ray resolution used for predictions (see explanations for poses), if zero(not specified), then the same resolution as for information retrieval is used
float64 forecast_ray_resolution_y