
import visibility_graph

# NOTE: no class here...just import * from this package to pull these methods in
    
# MAIN ENTRYPOINTS - convert_obstacles, get_lanes
# Converts an ObstacleArrayStamped to Polygon structures
def convert_obstacles(self, obstacles):
    new_obstacles = []
    for obstacle in obstacles.obstacles:
        if obstacle.type == obstacle.TYPE_OBSTACLE:
            for old_polygon in obstacle.polygons:
                new_obstacles.append(convert_polygon(old_polygon))
    return new_obstacles

# Returns lanes where each lane is represented by intersection points with the world boundaries
def get_lanes(self, obstacles):
    lanes = []
    for obstacle in obstacles.obstacles:
        if obstacle.type == obstacle.TYPE_LANE:
            intersects = []
            for old_polygon in obstacles.polygons:
                new_polygon = convert_polygon(old_polygon)
                intersect = world_intersect(new_polygon)
                if intersect != None:
                    intersects.append(intersect)
            if len(intersects):
                lane.append(intersects)
    return lanes

# Returns the intersection of two edges 
def edge_intersect(edge1, edge2):
    intersect = None
    e1pts = edge1.points
    e2pts = edge2.points

    if e2pts[0].x == e2pts[1].x:
        if e1pts[0].x == e1pts[1].x:
            return None
        points_slope = (e1pts[0].y - e1pts[1].y) / (e1pts[0].x - e1pts[1].x)
        intersect_x = e2pts[0].x
        intersect_y = points_slope * (intersect_x - e1pts[0].x) + e1pts[0].y
        intersect = Point(intersect_x, intersect_y)
    elif e1pts[0].x == e1pts[1].x:
        edge_slope = (e2pts[0].y - e2pts[1].y) / (e2pts[0].x - e2pts[1].x)
        intersect_x = e1pts[0].x
        intersect_y = edge_slope * (intersect_x - e2pts[0].x) + e2pts[0].y
        intersect = Point(intersect_x, intersect_y)
    else:
        edge_slope = (e2pts[0].y - e2pts[1].y) / (e2pts[0].x - e2pts[1].x)
        points_slope = (e1pts[0].y - e1pts[1].y) / (e1pts[0].x - e1pts[1].x)
        if edge_slope == points_slope:
            return None
        intersect_x = (edge_slope * e2pts[0].x - points_slope * e1pts[0].x + e1pts[0].y - e2pts[0].y) / (edge_slope - points_slope)
        intersect_y = edge_slope * (intersect_x - e2pts[0].x) + e2pts[0].y
        intersect = Point(intersect_x, intersect_y)

    # return intersect on both lines
    if intersect in [e1pts[0], e1pts[1], e2pts[0], e2pts[1]]:
        return intersect
    
    if not ((intersect.x <= e1pts[0].x and intersect.x >= e1pts[1].x) or (intersect.x >= e1pts[0].x and intersect.x <= e1pts[1].x)):
        return None
    if not ((intersect.y <= e1pts[0].y and intersect.y >= e1pts[1].y) or (intersect.y >= e1pts[0].y and intersect.y <= e1pts[1].y)):
        return None
    if not ((intersect.x <= e2pts[0].x and intersect.x >= e2pts[1].x) or (intersect.x >= e2pts[0].x and intersect.x <= e2pts[1].x)):
        return None
    if not ((intersect.y <= e2pts[0].y and intersect.y >= e2pts[1].y) or (intersect.y >= e2pts[0].y and intersect.y <= e2pts[1].y)):
        return None
    return intersect

# Converts a single Obstacle to a Polygon structure
def convert_polygon(old_polygon):
    new_points = []
    new_edges = []
    for point in old_polygon.points:
        new_points.append(visibility_graph.Point(point.x, point.y))
    for i in xrange(len(new_points) - 1):
        new_edges.append(visibility_graph.Edge(new_points[i], new_points[i + 1]))
    if len(new_points) > 2:
        new_edges.append(visibility_graph.Edge(new_points[len(new_points) - 1], new_points[0]))
    return visibility_graph.Polygon(new_points, new_edges)

def world_intersect(self, polygon):
    a = visibility_graph.Edge(visibility_graph.Point(-WORLD_VIEW_WIDTH/2, 0), visibility_graph.Point(-WORLD_VIEW_WIDTH/2, WORLD_VIEW_HEIGHT))
    b = visibility_graph.Edge(visibility_graph.Point(-WORLD_VIEW_WIDTH/2, WORLD_VIEW_HEIGHT), visibility_graph.Point(WORLD_VIEW_WIDTH/2, WORLD_VIEW_HEIGHT))
    c = visibility_graph.Edge(visibility_graph.Point(WORLD_VIEW_WIDTH/2, WORLD_VIEW_HEIGHT), visibility_graph.Point(WORLD_VIEW_WIDTH/2, 0))
    d = visibility_graph.Edge(visibility_graph.Point(WORLD_VIEW_WIDTH/2, 0), visibility_graph.Point(-WORLD_VIEW_WIDTH/2, 0))
    world_edges = [a, b, c, d]
    for edge in polygon.edges:
        for wedge in world_edges:
            intersect = edge_intersect(edge, wedge)
            if intersect != None:
                return intersect
    return None


