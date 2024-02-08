int16   agent_id
float32 agent_radius
# the trajectory id, starts from "1".
uint32 traj_id
uint8 order
uint8 dim
time start_time

# the polynomial coecfficients of the trajectory.
float64 start_yaw
float64 final_yaw
float64[] bound_x
float64[] bound_y
float64[] bound_z
float32[] duration
