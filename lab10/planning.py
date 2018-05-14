
#author1:
#author2:
import time

from grid import *
from visualizer import *
import threading
import operator
import math
import cozmo

from cozmo.util import degrees, distance_mm, speed_mmps
from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    start = grid.getStart()
    goal = grid.getGoals()[0]

    closed = []
    open = [start]

    reverseLookup = {}
    gScore = {start:0}
    fScore = {start:heuristic(start, goal)}

    while open:
        for entry in sorted(fScore.items(), key=operator.itemgetter(1)):
            if entry[0] in open:
                current = entry[0]
                break

        if current == goal:
            grid.setPath(getPath(reverseLookup, goal))
            break

        open.remove(current)
        closed.append(current)
        grid.addVisited(current)

        for neighbor in grid.getNeighbors(current):
            neighbor = neighbor[0]
            if neighbor in closed:
                continue

            if neighbor not in open:
                open.append(neighbor)

            tentativeG = gScore.get(current, math.inf) + euclideanDistance(current, neighbor)
            if tentativeG >= gScore.get(neighbor, math.inf):
                continue

            reverseLookup[neighbor] = current
            gScore[neighbor] = tentativeG
            fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal)

    pass # Your code here

def getPath(reverseLookup, goal):
    path = [goal]
    current = goal
    while current in reverseLookup:
        current = reverseLookup[current]
        path.append(current)
    path.reverse()
    return path

def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    return euclideanDistance(current, goal) # Simple euclidean distance heuristic

def euclideanDistance(start, end):
    return math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent

    # Reset the lift and head
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    robot.world.connect_to_cubes()
    robot.world.auto_disconnect_from_cubes_at_end()
    time.sleep(3) # Not sure why this helps but it does; the cube detection API is garbage

    # Look around and try to find a cube
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    robot.world.wait_for_observed_light_cube()
    look_around.stop()
    robot.turn_in_place(degrees(-robot.pose.rotation.angle_z.degrees)).wait_for_completed()

    origin = grid.getStart()
    scale = grid.scale

    curCell = origin
    curOrientation = 0
    goal_observed = None
    obstacle1_observed = None
    obstacle2_observed = None

    obstacles = {}
    while not stopevent.is_set():
        # Recalibrate where the goal is, if possible
        goal = robot.world.get_light_cube(LightCube1Id) # paperclip
        if goal.last_observed_time != goal_observed:
            goal_observed = goal.last_observed_time
            grid.clearGoals()
            goal_offset = offsetFromCubeAngle(goal.pose.rotation.angle_z.degrees)
            goal_grid = objectToGridCoords(goal, scale, origin)
            obstacles['goal'] = padObject(goal_grid, grid)
            grid.clearObstacles()
            grid.addObstacles([item for sublist in obstacles.values() for item in sublist])
            goal_grid = (goal_grid[0]+goal_offset[0], goal_grid[1]+goal_offset[1])
            grid.addGoal(goal_grid)

        # Recalibrate where the obstacles are, if possible
        obstacle1 = robot.world.get_light_cube(LightCube2Id) # heart
        if (obstacle1 and obstacle1.last_observed_time != obstacle1_observed):
            obstacle1_observed = obstacle1.last_observed_time
            obstacles['obstacle1'] = padObject(objectToGridCoords(obstacle1, scale, origin), grid)
            grid.clearObstacles()
            grid.addObstacles([item for sublist in obstacles.values() for item in sublist])

        obstacle2 = robot.world.get_light_cube(LightCube3Id) # weird
        if (obstacle2 and obstacle2.last_observed_time != obstacle2_observed):
            obstacle2_observed = obstacle2.last_observed_time
            obstacles['obstacle2'] = padObject(objectToGridCoords(obstacle2, scale, origin), grid)
            grid.clearObstacles()
            grid.addObstacles([item for sublist in obstacles.values() for item in sublist])

        # Recalculate the path to take
        grid.clearPath()
        grid.setStart(curCell)
        astar(grid, heuristic)

        # Make the first move along the calculated path
        if (len(grid.getPath())>2):
            toCell = grid.getPath()[1]
            curOrientation = move(robot, curCell, curOrientation, toCell, grid)
            #curCell = objectToGridCoords(robot, scale, origin)
            curCell = toCell

def move(robot, curCell, curOrientation, toCell, grid):
    offset = (toCell[0]-curCell[0], toCell[1]-curCell[1])
    angle = angleFromOffset(offset)
    turn = angle-curOrientation
    if turn>180:
        turn = turn-360
    elif turn<-180:
        turn = turn+360

    robot.turn_in_place(degrees(turn)).wait_for_completed()
    mm = euclideanDistance((0,0), offset) * grid.scale
    robot.drive_straight(distance_mm(mm), speed_mmps(mm)).wait_for_completed()
    return angle

def angleFromOffset(offset):
    return {
        (1,0): 0,
        (1,1): 45,
        (0,1): 90,
        (-1,1): 135,
        (-1,0): 180,
        (-1,-1): 225,
        (0,-1): 270,
        (1,-1): 315
    }[offset]

def offsetFromCubeAngle(angle):
    scalingFactor = 3
    if angle>=-22.5 and angle<22.5:
        return (-1*scalingFactor, 0*scalingFactor)
    elif angle>=22.5 and angle<67.5:
        return (-1*scalingFactor, -1*scalingFactor)
    elif angle>=67.5 and angle<112.5:
        return (0*scalingFactor, -1*scalingFactor)
    elif angle>=112.5 and angle<157.5:
        return (1*scalingFactor, -1*scalingFactor)

    elif angle<-22.5 and angle>=-67.5:
        return (-1*scalingFactor, 1*scalingFactor)
    elif angle<-67.5 and angle>=-112.5:
        return (0*scalingFactor, 1*scalingFactor)
    elif angle<-112.5 and angle>=-157.5:
        return (1*scalingFactor, 1*scalingFactor)
    else:
        return (1*scalingFactor, 0*scalingFactor)

def padObject(objCoords, grid):
    obstacles = []
    for i in range(-2, 3, 1):
        for j in range(-2, 3, 1):
            cell = (objCoords[0] + i, objCoords[1] + j)
            if grid.coordInBounds(cell) and cell not in obstacles:
                obstacles.append(cell)
    return obstacles

def objectToGridCoords(lightCube, scale, origin):
    x = lightCube.pose.position.x // scale + origin[0]
    y = lightCube.pose.position.y // scale + origin[1]
    return (x, y)

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

