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

# Global allowed error for movement
MAX_ERROR = 0

# Global publisher for the path
path_pub = None

# Global state holders
brain_state = brain_state.BrainState()
world_model = world_model.WorldModel()
path_planner = path_planner.PathPlanner()

def node():
    rospy.init_node('planner')
    path_pub = rospy.Publisher('planned_path', Path)
    rospy.Subscriber('obstacles', ObstacleArrayStamped, update_obstacles)
    # TODO assuming type here
    rospy.Subscriber('next_waypoint', Point32, update_destination)
    # TODO GPS, but I don't know the msg type
    # rospy.Subscriber('fix', ???, update_position)
    rospy.Subscriber('vel', Float64, update_velocity)
    print "Planner active"
    rospy.spin()

def update_obstacles(obstacles_data):
	global world_model, brain_state, path_planner
	obstacles = obstacles_data.data.obstacles
	new_time = obstacles_data.data.header.stamp

	dest_dir = 0
	dx = brain_state.current_destination.x - brain_state.current_location.x
	dy = brain_state.current_destination.y - brain_state.current_location.y
	if dx == 0:
		if current_location.x < current_destination.x:
			dest_dir = 0
		else:
			dest_dir = math.pi
	else:
		dest_dir = math.atan(dy / dx)
		if current_location.x > current_destination.x:
			dest_dir += math.pi
		elif current_location.y > current_destination.y:
			dest_dir += math.pi * 2

	plan = None
	if movement_validator.movement_valid(world_model.velocity, new_time - brain_state.last_update_time, world_model.obstacles, obstacles, MAX_ERROR):
		plan = path_planner.update_plan(obstacles, brain_state.last_update_time, brain_state.velocity, dest_dir)
	else:
		plan = path_planner.plan_new_path(obstacles, dest_dir)

	world_model.obstacles = obstacles
	brain_state.last_update_time = new_time

	if plan != None:
		path_pub.publish(plan)

def update_destination(destination_data):
	brain_state.current_destination = destination_data.data

	dest_dir = 0
	dx = brain_state.current_destination.x - brain_state.current_location.x
	dy = brain_state.current_destination.y - brain_state.current_location.y
	if dx == 0:
		if current_location.x < current_destination.x:
			dest_dir = 0
		else:
			dest_dir = math.pi
	else:
		dest_dir = math.atan(dy / dx)
		if current_location.x > current_destination.x:
			dest_dir += math.pi
		elif current_location.y > current_destination.y:
			dest_dir += math.pi * 2

	path_pub.publish(path_planner.plan_new_path(world_model.obstacles, dest_dir))

def update_velocity(velocity_data):
	brain_state.velocity = velocity_data.data

def update_position(position_data):
	# TODO format needed
	pass

if __name__ == "__main__":
    node()
