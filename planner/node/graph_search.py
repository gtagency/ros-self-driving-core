import math
import heapq

def distance(point1, point2):
    return math.sqrt(math.pow(point1.x - point2.x, 2) + math.pow(point1.y - point2.y, 2))

def heursitic(point, goal):
    return distance(point, goal)

def getSuccessors(point, edges):
    successors = []
    for edge in edges:
        if edge.contains(point):
            if edge.points[0] == point:
                successors.append(edge.points[1])
            else:
                successors.append(edge.points[0])
    return successors


# A* search that returns the path to the farthest point towards the goal if no other exists
def graph_search(points, edges, start, goal):
    current_solution = []
    current_closest_distance = float("inf")

    pqueue = []
    visited = set([])
    heapq.heappush(pqueue, (0 + heuristic(start, goal), start, [start], 0))

    while not len(pqueue) == 0 :
        state = heapq.heappop(pqueue)

        if state[1] not in visited :
            visited.add(state[1])
            
            if state[1] == goal :
                return state[2]
            
            if heuristic(state[1], goal) < current_closest_distance:
                current_closest_distance = heuristic(state[1], goal)
                current_solution = state[2]

            for succ in getSuccessors(state[1], edges) :
                if succ not in visited :
                    moveList = list(state[2])
                    moveList.append(succ)
                    pathCost = state[3] + distance(state[1], succ)
                    heapq.heappush(pqueue, (pathCost + heuristic(succ, goal), succ, moveList, pathCost))

    return solution