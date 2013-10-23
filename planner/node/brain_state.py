# brain_state.py

class StampedValue:
    value = None
    last_update_time = None

    def update(self, val, time):
        self.value = val
        self.last_update_time = time

class BrainState:
	current_location    = StampedValue()
	current_orientation = StampedValue()
	velocity            = StampedValue()
	current_destination = StampedValue()

    def getDistanceToGoal(self):
        if current_destination and current_location:
    	    dx = self.current_destination.value.longitude - self.current_location.value.longitude
	        dy = self.current_destination.value.latitude  - self.current_location.value.latitude
            return (dx, dy)
        else:
            return None

    def updateCurrentDestination(self, destination, time):
        self.current_destination.update(destination, time)
    
    def updateCurrentLocation(self, location, time):
        self.current_location.update(location, time)
    
    def updateVelocity(self, velocity, time):
        self.current_location.update(velocity, time)
