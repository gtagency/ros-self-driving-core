#!/usr/bin/env python
import roslib
import rospy
import math
import path_planner
import movement_validator
import world_model
import brain_state
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import Point32
from sensor_msgs.msg import NavSatFix

import visibility_graph

# Global allowed error for movement
MAX_ERROR = 0

# Global publisher for the path
path_pub = None

# Global state holders
brain_state = brain_state.BrainState()
world_model = world_model.WorldModel()
path_planner = path_planner.PathPlanner(visibility_graph.VisibilityGraphPlanningStrategy())

class GlobalPoint:
    latitude = None
    longitude = None

    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

def node():
    rospy.init_node('planner')
    path_pub = rospy.Publisher('planned_path', Path)
    rospy.Subscriber('obstacles', ObstacleArrayStamped, update_obstacles)
    rospy.Subscriber('next_waypoint', Point, update_destination)
    rospy.Subscriber('fix', NavSatFix, update_position)
    rospy.Subscriber('vel', Float64, update_velocity)
    rospy.loginfo("Planner active")
    rospy.spin()

def calc_destination_direction():
    global brain_state
    dest_dir = 0
	(dx, dy) = brain_state.getDistanceToGoal()
#	
#	if dx == 0:
#		if current_location.longitude < current_destination.latitude:
#			dest_dir = 0
#		else:
#			dest_dir = math.pi
#	else:
#		dest_dir = math.atan(dy / dx)
#		if current_location.longitude > current_destination.longitude:
#			dest_dir += math.pi
#		elif current_location.latitude > current_destination.latitude:
#			dest_dir += math.pi * 2

    dest_dir = math.atan2(dy, dx)
    # wrap negative angles around to be between 0 and pi
    if dest_dir < 0:
        dest_dir = math.pi + dest_dir
    return dest_dir

def update_obstacles(obstacles_data):
	global world_model, brain_state, path_planner

	obstacles = convert_obstacles(obstacles_data.data.obstacles)
    lanes     = get_lanes(obstacles_data.data.obstacles)
	new_time = obstacles_data.data.header.stamp

    model_delta = world_model.update(obstacles, lanes, new_time)
	dest_dir = calc_destination_direction()

	plan = None
    # TODO: why is this separate from the path_planner?
	if movement_validator.movement_valid(brain_state.velocity.value, model_delta, MAX_ERROR):
		plan = path_planner.update_plan(model_delta, dest_dir)
	else:
		plan = path_planner.plan_new_path(world_model, dest_dir)

	if plan != None:
		path_pub.publish(plan)

def update_destination(destination_data):
	gpt = GlobalPoint(destination_data.data.y, destination_data.data.x)
    brain_state.updateCurrentDestination(gpt, destination_data.data.header.stamp)

	dest_dir = calc_destination_direction()

	path_pub.publish(path_planner.plan_new_path(world_model.obstacles, dest_dir))

def update_velocity(velocity_data):
	brain_state.updateVelocity(velocity_data.data, position_data.data.header.stamp

def update_position(position_data):
	gpt = GlobalPoint(position_data.data.y, position_data.data.x)
	brain_state.updateCurrentLocation(gpt, position_data.data.header.stamp

if __name__ == "__main__":
    node()
