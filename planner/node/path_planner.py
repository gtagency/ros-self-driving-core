# path_planner.py

class PathPlanner:
	plan = None

	# Forces a replan
	# Requires the obstacles to plan around
	# Uses the direction of the final destination
	def plan_new_path(obstacles, final_dest_dir):
		plan = None
		return plan

	# May or may not update the plan
	# Returns 0 if no update is made
	def update_plan(obstacles, curr_time, velocity, final_dest_dir): #IDONTEVENKNOW
		# Translate path to current position
		# If path still valid and long enough, continue
		# Otherwise replan
		
		if (True):
			return plan_new_path(obstacles)