Header header
uint32 width
uint32 height
# The map data, in row-major order, starting with (0,0). 
# For element (x,y) access as: cell_occupancy_type[index]
#		with index = y*(width) + x
int8 cell_xsize
int8 cell_ysize
int8[] cell_occupancy_type
int8[] action_policy_type
int32[] robot_local_cell_indices
int8[] state_type
float64[] state_distance
int32[] robot_global_cell_indices


