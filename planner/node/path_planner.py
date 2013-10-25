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
	def plan_new_path(world_model, brain_state, final_dest_dir):
        # TODO: determining the goal is probably best put in the planning strategy too
		goal = determine_goal(world_model, brain_state, final_dest_dir)
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

	def determine_goal(world_model, brain_state, final_dest_dir):
        # Strategy: sample from the furthest polygon that defines the drivable space, and pick the one that fits most evenly between all of the obstacles around it
        # NEED: lane_detect to report 1-n polygons about what is drivable.  This assumes curves will be overlapping polygons, and will pick a goal that's between them (kind of like an apex of a turn)

        last_goal = brain_state.last_goal if brain_state.last_goal != None else brain_state.current_location
        surfaces = world_model.get_drivable_surfaces()
        # extend last_goal in the direction the car may be traveling in 
        numPoints = 100
        for i in xrange(numPoints):
            # the score is the sum of distance from the containing polygon to all adjacent polygons (centers).  this should, in the case of a curve, put the ideal goal at the intersection of the near poly and far poly
            (possible_goal, dist_from_center) = sample_from_and_Score(surfaces, brain_state.current_location.value, brain_state.velocity.value)
            score = dist_from_
        brain_state.last_goal = goal
		return Point(0, world_model.world_view_height)

	def curve_plan():
		# TODO
		return None
