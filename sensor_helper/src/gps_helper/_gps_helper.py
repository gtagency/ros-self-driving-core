
import math

class GpsHelper(object):

    def __init__(self, debug=False):
        self.latitude = None
        self.longitude = None
        self.covariance = None
        self.debug = debug

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371 # km
        degToRad = math.pi / 180.0
        dLat = (lat2-lat1) * degToRad
        dLon = (lon2-lon1) * degToRad
        lat1 = lat1 * degToRad
        lat2 = lat2 * degToRad;

        a = math.sin(dLat/2) * math.sin(dLat/2) + \
                math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2) 
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c

        return d * 1000.0 # convert to meters

    def atPoint(self, latitude, longitude):
        if not latitude or \
           not longitude or \
           not self.isSet():
            if self.debug:
                rospy.loginfo("Not enough info to determine if at point")
            return False
        
        # test if we're within the error bounds of the waypoint..if so, we're 'near' it
        # SUPER CHEAT because I know the GPS data is coming from a phone, and therefore
        # I don't really have a full covariance matrix.  This pulls out the 68% accurac
        # computed by the phone, which is 1 std dev from the estimated position
        # TODO Need to figure out how to compute error better, for nearCurrentWaypoint
        errorInMeters = math.sqrt(self.covariance[0])
        dist = abs(self.haversine(latitude, longitude, self.latitude, self.longitude))
        if self.debug:
            rospy.loginfo("Error: %f, Dist: %f" % (errorInMeters, dist))
        return dist < errorInMeters

    def distanceAndBearingTo(self, latitude, longitude):
        distance = abs(self.haversine(latitude, longitude, self.latitude, self.longitude))
        degToRad = math.pi / 180.0
        dLonRad = (longitude - self.longitude) * degToRad
        dLatRad = (latitude - self.latitude) * degToRad
#        lat2Rad = latitude * degToRad
 #       lat1Rad = self.latitude * degToRad
  #      y = math.sin(dLonRad) * math.cos(lat2Rad);
   #     x = math.cos(lat1Rad) * math.sin(lat2Rad) -\
    #        math.sin(lat1Rad) * math.cos(lat2Rad) * math.cos(dLonRad)
     #   print y, x
      #  brng = math.atan2(y, x)
        brng = math.atan2(dLatRad, dLonRad)
        return (distance, brng)
 
    def isSet(self):
        return self.latitude and self.longitude and  self.covariance

    def update(self, latitude, longitude, covariance):
        self.latitude = latitude
        self.longitude = longitude
        self.covariance = list(covariance) #3x3 matrix in a 9 element array
        if self.debug:
            rospy.loginfo("Updated position: %f, %f" % (self.latitude, self.longitude))
    
