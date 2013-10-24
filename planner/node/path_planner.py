# path_planner.py

from world_model import *

CURVE_RADIUS = 6
CAR_WIDTH = 2

class PathPlanner:
	plan = None

    def __init__(self, planning_strategy):
        self.planning_strategy = planning_strategy

	# Forces a replan
	# Requires the obstacles to plan around
	# Uses the direction of the final destination
	def plan_new_path(world_model, final_dest_dir):
        # TODO: determining the goal is probably best put in the planning strategy too
		goal = determine_goal(world_model, final_dest_dir)
		start = self.planning_strategy.init_start()
		plan = self.planning_strategy.plan(world_model.obstacles, start, goal, CURVE_RADIUS + CAR_WIDTH)
		curve_plan()
		return plan

	# May or may not update the plan
	# Returns 0 if no update is made
	def update_plan(obstacles, curr_time, velocity, final_dest_dir): #IDONTEVENKNOW
		# Translate path to current position
		# If path still valid and long enough, continue
		# Otherwise replan
		# TODO
		if (True):
			return self.plan_new_path(obstacles, final_dest_dir)

	def determine_goal(world_model, final_dest_dir):
		# TODO
		return Point(0, world_model.world_view_height)

	def curve_plan():
		# TODO
		return None
