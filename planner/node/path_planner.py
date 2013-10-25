# path_planner.py

from world_model import *
import math

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
		points = plan
		corner_angles = []
		for i in xrange(len(points) - 2):
			point_start = points[i]
			point_mid = points[i + 1]
			point_end = points[i + 2]
			adx = point_mid.x - point_start.x
			ady = point_mid.y - point_start.y
			bdx = point_mid.x - point_end.x
			bdy = point_mid.y - point_end.y
			adotb = adx * bdx + ady * bdy
			atimesb = math.sqrt((adx * adx + ady * ady) * (bdx * bdx + bdy * bdy))
			corner_angles.append(math.acos(adotb / atimesb))
		distance_cuts = [ CURVE_RADIUS * math.sin((math.pi - a) / 2) / math.sin(math.pi / 2) for a in corner_angles ]

		# Eliminating later sections of the path if sections are too small to turn through
		if distance_cuts[0] > distance(points[0], points[1]):
			plan = []
			return
		for i in xrange(len(p) - 3):
			if distance_cuts[i] + distance_cuts[i + 1] > distance(point[i + 1], point[i + 2]):
				points = points[:i + 3]
				distance_cuts = distance_cuts[:i + 1]
				corner_angles = corner_angles[:i + 1]
				break
		if distance_cuts[len(distance_cuts) - 1] > distance(points[len(points) - 2], points[len(points) - 1]):
			points = points[:len(points) - 1]
			distance_cuts = distance_cuts[:len(distance_cuts) - 1]
			corner_angles = corner_angles[:len(corner_angles) - 1]

		plan = []
		plan.append((distance(points[0], points[2]) - distance_cuts[0], 0, CURVE_RADIUS))
		for i in xrange(len(corner_angles) - 1):
			s = CURVE_RADIUS * (math.pi - corner_angles[i])
			plan.append((s, corner_angles[i], CURVE_RADIUS))
			plan.append((distance(point[i + 1], point[i + 2]) - distance_cuts[i] - distance_cuts[i + 1], 0, CURVE_RADIUS))
		s = CURVE_RADIUS * (math.pi - corner_angles[len(corner_angles) - 1])
		plan.append((s, corner_angles[len(corner_angles) - 1], CURVE_RADIUS))
		plan.append((distance(points[len(points) - 2], points[len(points) - 1]) - distance_cuts[len(distance_cuts) - 1], 0, CURVE_RADIUS))


	def distance(point1, point2):
		return math.sqrt(math.pow(point1.x - point2.x, 2) + math.pow(point1.y - point2.y, 2))
