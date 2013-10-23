# path_planner.py

import visibility_graph
from graph_search import graph_search
from world_model import *

CURVE_RADIUS = 6
CAR_WIDTH = 2
WORLD_VIEW_WIDTH = 30
WORLD_VIEW_HEIGHT = 30

class PathPlanner:
	plan = None

	# Forces a replan
	# Requires the obstacles to plan around
	# Uses the direction of the final destination
	def plan_new_path(obstacles, final_dest_dir):
		goal = determine_goal(get_lanes(obstacles), final_dest_dir)
		# TODO this should be done by the world model, and the planner should plan on already processed obstcles
        polygons = convert_obstacles(obstacles)
		start = visibility_graph.Point(0, 0)
		graph = visibility_graph.visibility_graph(polygons, start, goal, CURVE_RADIUS + CAR_WIDTH)
		plan = graph_search(graph[0], graph[1], start, goal)
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

	def determine_goal(polygons, final_dest_dir):
		# TODO
		return Point(0, WORLD_VIEW_HEIGHT)

	def curve_plan():
		# TODO
		return None
