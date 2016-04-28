# the pose...
geometry_msgs/Pose pose

# pose is relative to this frame (as specified in tf), that is the transform that could be used to transform a poit in the pose frame into one in the source frame
string source_frame

# whether the view is reachable or not
bool is_reachable

# marks a bad view for whatever reason (e.g. data reception failed here or the like)
bool is_bad

# how many times this view has been visited
uint32 visited

# index
uint32 index

# additional fields names
string[] associated_names
# additional fields values
float64[] associated_values