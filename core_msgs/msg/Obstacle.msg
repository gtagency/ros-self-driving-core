#
# Obstacle messages are used to report any non-drivable object or region
# detected at a given time.
#
#

# Message header, which includes a time stamp
Header header

# A set of polygons to describe the obstacle.  It is assumed that
# the walls and interior of all of the polygons are non-drivable.
Polygon polygons[]
