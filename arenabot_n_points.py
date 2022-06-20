# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import time

import inspyred
import random
import numpy as np
from shapely.geometry import Polygon, LineString, Point
import arenabot_lib as lib
import math

random_number_generator = random.Random()
random_number_generator.seed(
    time.time())  # seeding the generators with a fixed value ensures that
# you will always obtain the same sequence of numbers at every run

walls = []

# the Arena is a 100 x 100 pixel space
arenaLength = 100
arenaWidth = 100

# let's also put a couple of walls in the arena; walls are described by a set of 4 (x,y)
# corners (bottom-left, top-left, top-right, bottom-right)
wall1 = dict()
wall1["x"] = 30
wall1["y"] = 0
wall1["width"] = 10
wall1["height"] = 80

wall2 = dict()
wall2["x"] = 70
wall2["y"] = 20
wall2["width"] = 10
wall2["height"] = 80

walls.append(wall1)
walls.append(wall2)

labyrinthe_wall1 = Polygon([(wall1["x"], wall1["y"]), (wall1["x"] + wall1["width"], wall1["y"]),
                            (wall1["x"] + wall1["width"], wall1["y"] + wall1["height"]),
                            (wall1["x"], wall1["y"] + wall1["height"])])
labyrinthe_wall2 = Polygon([(wall2["x"], wall2["y"]), (wall2["x"] + wall2["width"], wall2["y"]),
                            (wall2["x"] + wall2["width"], wall2["y"] + wall2["height"]),
                            (wall2["x"], wall2["y"] + wall2["height"])])
labyrinthe_line = [(0, 0), (0, arenaLength), (arenaWidth, arenaLength), (arenaWidth, 0), (0, 0)]
labyrinthe_arena = LineString(labyrinthe_line)


def fitnessRobot(listOfCommands, args):
    walls = args["walls"]
    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]
    objectiveX = args["objectiveX"]
    objectiveY = args["objectiveY"]

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
    if distanceLine.intersects(labyrinthe_wall1):
        WallsBeforeEnd += MalusFromHorizon
    if distanceLine.intersects(labyrinthe_wall2):
        WallsBeforeEnd += MalusFromHorizon

    # for each time the robot goes through a wall, a malus is added
    malusGain = 100
    closeToWallMalus = 0  # init malus to 0

    for k in range(len(positions) - 1):  # add malus everytime the path encounters a wall,
        # when it encounters a wall, the nearer it is from the corner the better it is
        currentLine = LineString([(positions[k]), (positions[k + 1])])
        if currentLine.intersects(labyrinthe_wall1):
            minDistanceFromCorner = min([currentLine.distance(Point(x)) for x in
                                         [(wall1["x"], wall1["y"]), (wall1["x"] + wall1["width"], wall1["y"]),
                                          (wall1["x"] + wall1["width"], wall1["y"] + wall1["height"]),
                                          (wall1["x"], wall1["y"] + wall1["height"])]])
            closeToWallMalus += malusGain + minDistanceFromCorner
        if currentLine.intersects(labyrinthe_wall2):
            minDistanceFromCorner = min([currentLine.distance(Point(x)) for x in
                                         [(wall2["x"], wall2["y"]), (wall2["x"] + wall2["width"], wall2["y"]),
                                          (wall2["x"] + wall2["width"], wall2["y"] + wall2["height"]),
                                          (wall2["x"], wall2["y"] + wall2["height"])]])
            closeToWallMalus += malusGain + minDistanceFromCorner

    fitness = distanceFromObjective + closeToWallMalus + WallsBeforeEnd

    return fitness


def evaluate(candidates, args):
    scores = []

    for candidate in candidates:
        scores.append(fitnessRobot(candidate, args))

    return scores


# this function generates initial random commands for the robot; it needs a random number
# generator (called "random") and a dictionary or arguments in input,
# to be used by the EvolutionaryComputation object; returns one (randomly generated) candidate individual
def generator_commands(random, args):
    minimum_angle = args["minimum_angle"]  # also, the minimum value of each dimension will be specified later in "args"
    maximum_angle = args["maximum_angle"]  # same goes for the maximum value

    minimum_distance = args[
        "minimum_distance"]  # also, the minimum value of each dimension will be specified later in "args"
    maximum_distance = args["maximum_distance"]  # same goes for the maximum value

    max_individual_length = args["max_individual_length"]
    min_individual_length = args["min_individual_length"]

    # the individual will be a series of "number_of_dimensions" random values, generated between "minimum" and "maximum"
    # the individual will be a series of commands (move and rotate)

    individual_length = random.randint(min_individual_length, max_individual_length) // 2 * 2
    individual = np.zeros(individual_length)

    for i in range(0, individual_length // 2):
        individual[2 * i] = random.uniform(minimum_angle, maximum_angle)
        individual[2 * i + 1] = random.uniform(minimum_distance, maximum_distance)

    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]

    _, _, _, positions = lib.applicateCommands(startX, startY, startDegrees, individual)

    if len(positions) > 1:
        line = LineString(positions)
        if line.intersects(labyrinthe_arena):
            return generator_commands(random, args)

    return individual


def insertion(random, candidates, args):
    insertion_rate = args.get('insertion_rate', 0.1)
    mutants = []
    for c in candidates:
        if random.uniform(0, 1) < insertion_rate:
            c2 = random.choice(candidates)

            insertion_position = random.randint(0, len(c) // 2) * 2
            insertion_start = random.randint(0, len(c2) // 2 - 1) * 2
            insertion_len = random.randint(0, (len(c2) - insertion_start) // 2) * 2

            c1 = np.zeros(len(c) + insertion_len)

            c1[0:insertion_position] = c[0:insertion_position]
            c1[insertion_position:insertion_position + insertion_len] = \
                c2[insertion_start:insertion_start + insertion_len]
            c1[insertion_position + insertion_len: len(c) + insertion_len] = c[insertion_position:len(c)]

            mutants.append(c1)

    return mutants


# ================ MAIN
def main():
    global walls

    startX = 10
    startY = 10
    startDegrees = 90
    objectiveX = 90
    objectiveY = 90

    # instantiate the evolutionary algorithm object
    evolutionary_algorithm = inspyred.ec.EvolutionaryComputation(random_number_generator)
    # and now, we specify every part of the evolutionary algorithm
    evolutionary_algorithm.selector = inspyred.ec.selectors.tournament_selection  # by default, tournament selection has tau=2 (two individuals), but it can be modified (see below)
    evolutionary_algorithm.variator = [insertion,
                                       inspyred.ec.variators.n_point_crossover,
                                       inspyred.ec.variators.gaussian_mutation]  # the genetic operators are put in a list, and executed one after the other
    evolutionary_algorithm.replacer = inspyred.ec.replacers.plus_replacement  # "plus" -> "mu+lambda"
    evolutionary_algorithm.terminator = inspyred.ec.terminators.evaluation_termination  # the algorithm terminates when a given number of evaluations (see below) is reached

    # evolutionary_algorithm.observer = inspyred.ec.observers.best_observer # prints best individual to screen
    # evolutionary_algorithm.observer = inspyred.ec.observers.stats_observer # print out population statistics
    evolutionary_algorithm.observer = lib.observer  # plots evolution
    final_population = evolutionary_algorithm.evolve(
        generator=generator_commands,  # of course, we need to specify the evaluator
        evaluator=evaluate,  # and the corresponding evaluator
        pop_size=1500,  # size of the population
        num_selected=700,  # size of the offspring (children individuals)
        maximize=False,  # this is a minimization problem, but inspyred can also manage maximization problem
        max_evaluations=100000,  # maximum number of evaluations before stopping, used by the terminator
        tournament_size=2,
        # size of the tournament selection; we need to specify it only if we need it different from 2
        crossover_rate=1.0,  # probability of applying crossover
        num_crossover_points=3,  # number of crossover point used
        mutation_rate=4,  # probability of applying mutation
        insertion_rate=0.1,
        # all arguments specified below, THAT ARE NOT part of the "evolve" method, will be automatically placed in "args"
        min_individual_length=20,
        max_individual_length=50,  # number of dimensions of the problem, used by "generator_weierstrass"
        minimum_angle=-90,  # minimum angle for generator_commands
        maximum_angle=90,  # maximum angle
        minimum_distance=0,  # minimum distance for generator_commands
        maximum_distance=30,  # maximum distance

        walls=walls,
        startX=startX,
        startY=startY,
        startDegrees=startDegrees,
        objectiveX=objectiveX,
        objectiveY=objectiveY,
        arena=labyrinthe_line
    )

    # after the evolution is over, the resulting population is stored in "final_population";
    # the best individual is on the top
    best_individual = final_population[0]
    print("The best individual has fitness %.2f" % best_individual.fitness)

    lib.plot_generation(final_population, walls, [startX, startY, startDegrees], [objectiveX, objectiveY])

    return 0


if __name__ == "__main__":
    sys.exit(main())
