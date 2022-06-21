import arenabot_lib as lib
import math
from shapely.geometry import Polygon, LineString, Point


def mainFitnessRobot(listOfCommands, args):
    walls = args["walls"]
    arena = args["arena"]
    labyrinthe_arena = LineString(arena)
    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]
    objectiveX = args["objectiveX"]
    objectiveY = args["objectiveY"]

    polygon_walls = []

    for wall in walls:
        polygon_walls.append(Polygon([(wall["x"], wall["y"]), (wall["x"] + wall["width"], wall["y"]),
                                      (wall["x"] + wall["width"], wall["y"] + wall["height"]),
                                      (wall["x"], wall["y"] + wall["height"])]))

    # this is a list of points that the robot will visit; used later to visualize its path
    robotX, robotY, robotDegrees, positions = lib.applicateCommands(startX, startY, startDegrees, listOfCommands)

    distanceFromObjective = math.sqrt((robotX - objectiveX) ** 2 + (robotY - objectiveY) ** 2)

    # move robot, check that the robot stays inside the arena and stop movement if a wall is hit
    # add a huge malus if the robot leaves the arena
    if len(positions) > 1:
        line = LineString(positions)
        intersectionArenaMalus = 1000
        if line.intersects(labyrinthe_arena):
            distanceFromObjective += intersectionArenaMalus * (1 + labyrinthe_arena.distance(Point(positions[-1])) / 20)

    # nb of walls between objective and final position of the robot
    MalusFromHorizon = 300
    WallsBeforeEnd = 0

    distanceLine = LineString([positions[-1], (objectiveX, objectiveY)])
    for labyrinthe_wall in polygon_walls:
        if distanceLine.intersects(labyrinthe_wall):
            WallsBeforeEnd += MalusFromHorizon

    # for each time the robot goes through a wall, a malus is added
    malusGain = 100
    closeToWallMalus = 0  # init malus to 0

    for k in range(len(positions) - 1):  # add malus everytime the path encounters a wall,
        # when it encounters a wall, the nearer it is from the corner the better it is
        currentLine = LineString([(positions[k]), (positions[k + 1])])

        for i in range(len(walls)):
            labyrinthe_wall = polygon_walls[i]
            wall = walls[i]
            if currentLine.intersects(labyrinthe_wall):
                minDistanceFromCorner = min([currentLine.distance(Point(x)) for x in
                                             [(wall["x"], wall["y"]), (wall["x"] + wall["width"], wall["y"]),
                                              (wall["x"] + wall["width"], wall["y"] + wall["height"]),
                                              (wall["x"], wall["y"] + wall["height"])]])
                closeToWallMalus += malusGain + minDistanceFromCorner

    fitness = distanceFromObjective + closeToWallMalus + WallsBeforeEnd

    return fitness


def uniformLuckyFitness(listOfCommands, args):
    walls = args["walls"]
    arena = args["arena"]
    labyrinthe_arena = LineString(arena)
    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]
    objectiveX = args["objectiveX"]
    objectiveY = args["objectiveY"]

    polygon_walls = []

    for wall in walls:
        polygon_walls.append(Polygon([(wall["x"], wall["y"]), (wall["x"] + wall["width"], wall["y"]),
                                      (wall["x"] + wall["width"], wall["y"] + wall["height"]),
                                      (wall["x"], wall["y"] + wall["height"])]))

    # this is a list of points that the robot will visit; used later to visualize its path
    robotX, robotY, robotDegrees, positions = lib.applicateCommands(startX, startY, startDegrees, listOfCommands)

    distanceFromObjective = math.sqrt((robotX - objectiveX) ** 2 + (robotY - objectiveY) ** 2)

    # add a huge malus if the robot leaves the arena
    if len(positions) > 1:
        line = LineString(positions)
        intersectionArenaMalus = 1000
        if line.intersects(labyrinthe_arena): distanceFromObjective += intersectionArenaMalus

    # nb of walls between objective and final position of the robot
    MalusFromHorizon = 300
    WallsBeforeEnd = 0

    distanceLine = LineString([positions[-1], (objectiveX, objectiveY)])
    for labyrinthe_wall in polygon_walls:
        if distanceLine.intersects(labyrinthe_wall):
            WallsBeforeEnd += MalusFromHorizon

    # calcule the length of the path
    LengthPath = 0

    # for each time the robot goes through a wall, a malus is added
    malusGain = 100
    closeToWallMalus = 0  # init malus to 0

    for k in range(len(positions) - 1):  # add malus everytime the path encounters a wall
        currentLine = LineString([(positions[k]), (positions[k + 1])])
        LengthPath += currentLine.length
        for labyrinthe_wall in polygon_walls:
            if currentLine.intersects(labyrinthe_wall):
                closeToWallMalus += malusGain

    fitness = distanceFromObjective + closeToWallMalus + WallsBeforeEnd

    return fitness


def firstFitness(listOfCommands, args):
    walls = args["walls"]
    arena = args["arena"]
    labyrinthe_arena = LineString(arena)
    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]
    objectiveX = args["objectiveX"]
    objectiveY = args["objectiveY"]

    polygon_walls = []

    for wall in walls:
        polygon_walls.append(Polygon([(wall["x"], wall["y"]), (wall["x"] + wall["width"], wall["y"]),
                                      (wall["x"] + wall["width"], wall["y"] + wall["height"]),
                                      (wall["x"], wall["y"] + wall["height"])]))

    # this is a list of points that the robot will visit; used later to visualize its path
    robotX, robotY, robotDegrees, positions = lib.applicateCommands(startX, startY, startDegrees, listOfCommands)

    distanceFromObjective = math.sqrt((robotX - objectiveX) ** 2 + (robotY - objectiveY) ** 2)

    # distance to the walls
    distanceThresholdForMalus = 5
    malusGain = 8
    closeToWallMalus = 0  # init malus to 0

    if len(positions) > 1:
        line = LineString(positions)

        intersectionMalus = 500

        if line.intersects(labyrinthe_arena):
            distanceFromObjective += intersectionMalus

        for labyrinthe_wall in polygon_walls:
            distanceFromWall = labyrinthe_wall.exterior.distance(line)

            # add malus if the closest point to each wall is below threshold if the distance to the wall is below a
            # threshold, the added malus to the fitness is greater as the path of the robot is close to the walls
            if distanceFromWall < distanceThresholdForMalus:
                closeToWallMalus += malusGain * (1 - distanceFromWall / distanceThresholdForMalus)

    fitness = distanceFromObjective + closeToWallMalus

    return fitness