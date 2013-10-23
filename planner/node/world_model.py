# world_model.py

class ModelDelta:
    old_obstacles = []
    new_obstacles = []
    delta_time    = None
    
class WorldModel:
	obstacles = []
	last_update_time = None

    def update(self, obstacles, update_time):
        delta_time = (update_time - last_update_time) if last_update_time else update_time
        delta = ModelDelta(self.obstacles, obstacles, delta_time)
        self.obstacles = obstacles
        self.last_update_time = update_time
        return delta 
        
