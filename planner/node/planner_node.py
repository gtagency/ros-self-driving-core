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

# Global allowed error for movement
MAX_ERROR = 0

# Global publisher for the path
path_pub = None

# Global state holders
brain_state = brain_state.BrainState()
world_model = world_model.WorldModel()
path_planner = path_planner.PathPlanner()

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
    print "Planner active"
    rospy.spin()

def calc_destination_direction():
    global brain_state
    dest_dir = 0
	dx = brain_state.current_destination.longitude - brain_state.current_location.longitude
	dy = brain_state.current_destination.latitude  - brain_state.current_location.latitude
	if dx == 0:
		if current_location.longitude < current_destination.latitude:
			dest_dir = 0
		else:
			dest_dir = math.pi
	else:
		dest_dir = math.atan(dy / dx)
		if current_location.longitude > current_destination.longitude:
			dest_dir += math.pi
		elif current_location.latitude > current_destination.latitude:
			dest_dir += math.pi * 2

    return dest_dir

def update_obstacles(obstacles_data):
	global world_model, brain_state, path_planner
	obstacles = obstacles_data.data.obstacles
	new_time = obstacles_data.data.header.stamp

	dest_dir = calc_destination_direction()

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
	brain_state.current_destination = GlobalPoint(destination_data.data.y, destination_data.data.x)

	dest_dir = calc_destination_direction()

	path_pub.publish(path_planner.plan_new_path(world_model.obstacles, dest_dir))

def update_velocity(velocity_data):
	brain_state.velocity = velocity_data.data

def update_position(position_data):
	brain_state.current_location = GlobalPoint(position_data.data.y, position_data.data.x)

if __name__ == "__main__":
    node()
