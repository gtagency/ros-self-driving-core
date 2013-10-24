# world_model.py

class ModelDelta:
    old_obstacles = []
    new_obstacles = []
    old_lanes     = []
    new_lanes     = []
    delta_time    = None
    
    def __init__(self, old_obstacles, new_obstacles, old_lanes, new_lanes, delta_time):
        self.old_obstacles = old_obstacles
        self.new_obstacles = new_obstacles
        self.old_lanes     = old_lanes
        self.new_lanes     = new_lanes
        self.delta_time    = delta_time

class WorldModel:
	obstacles = []
    lanes = []
	last_update_time = None

    world_view_width  = 30
    world_view_height = 30

    def update(self, obstacles, lanes, update_time):
        delta_time = (update_time - last_update_time) if last_update_time else update_time
        delta = ModelDelta(self.obstacles, obstacles, self.lanes, lanes, delta_time)
        self.obstacles = obstacles
        self.last_update_time = update_time
        return delta 
        
