# How many rays are cast per pixel on the image's x-axis to obtain the information. [rays/px] Default: 1.0
float64 ray_resolution_x
# How many rays are cast per pixel on the image's y-axis to obtain the information. [rays/px] Default: 1.0
float64 ray_resolution_y

# subwindow through which rays are cast (percentage of image size)
ig_active_reconstruction_msgs/SubWindow ray_window

# Maximal ray depth for the ig computation. [World representation units, usually m] Default: 10.0
float64 max_ray_depth