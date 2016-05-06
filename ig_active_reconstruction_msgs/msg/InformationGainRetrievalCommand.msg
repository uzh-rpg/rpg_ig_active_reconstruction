# returning the predicted informations for the last pose (given that the other poses were visited before)
geometry_msgs/Pose[] poses

# Vector with the names of all metrics that shall be calculated. Only considered if metric_ids is empty.
string[] metric_names

# Vector with the ids of all metrics that shall be calculated. Takes precedence over metric_names.
uint32[] metric_ids

# Configuration of information gain
ig_active_reconstruction_msgs/InformationGainRetrievalConfig config