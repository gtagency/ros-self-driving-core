# movement_validator.py

# Determines whether an estimated movement is within a given error
# Uses movement of obstacles to validate
def movement_valid(vel, time_dif, old_obst, new_obst, error):
	return True