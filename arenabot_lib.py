import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def moveRobot(robotX, robotY, robotDegrees, distance):
    robotX += distance * math.cos(robotDegrees * math.pi / 180)
    robotY += distance * math.sin(robotDegrees * math.pi / 180)
    return [robotX, robotY]


def rotateRobot(robotX, robotY, robotDegrees, angle):
    return robotDegrees + angle


def applicateCommands(robotX, robotY, robotDegrees, listOfCommands):
    number_of_commands = len(listOfCommands) // 2
    line_points = [(robotX, robotY)]

    for i in range(number_of_commands):
        robotDegrees = rotateRobot(robotX, robotY, robotDegrees, listOfCommands[2 * i])
        [robotX, robotY] = moveRobot(robotX, robotY, robotDegrees, listOfCommands[2 * i + 1])
        line_points.append((robotX, robotY))

    return robotX, robotY, robotDegrees, line_points


def plot_generation(population, walls, arena, start, objective, generation=None):
    figure = plt.figure()
    ax = figure.add_subplot(111)

    # plot a series of lines describing the movement of the robot in the arena
    for j in range(len(population)):
        listOfCommands = population[j].candidate
        robotX, robotY, robotDegrees, positions = applicateCommands(start[0], start[1], start[2], listOfCommands)
        for i in range(1, len(positions)):
            ax.plot([positions[i - 1][0], positions[i][0]], [positions[i - 1][1], positions[i][1]],
                    color='lightcoral')  # ,label="Robot path")

    print("Best : %f" % population[0].fitness)
    listOfCommands = population[0].candidate
    robotX, robotY, robotDegrees, positions = applicateCommands(start[0], start[1], start[2], listOfCommands)
    for i in range(1, len(positions)):
        ax.plot([positions[i - 1][0], positions[i][0]], [positions[i - 1][1], positions[i][1]],
                'r-')  # ,label="Robot path")

    # plot the walls
    for wall in walls:
        ax.add_patch(patches.Rectangle((wall["x"], wall["y"]), wall["width"], wall["height"]))

    # plot initial position and objective
    ax.plot(start[0], start[1], 'r^', label="Initial position of the robot")
    ax.plot(objective[0], objective[1], 'gx', label="Position of the objective")

    for i in range(1, len(arena)):
        ax.plot([arena[i - 1][0], arena[i][0]], [arena[i - 1][1], arena[i][1]],
                'b-')  # ,label="Robot path")

    if generation is None:
        ax.set_title("Movements of the robot inside the arena")
    else:
        ax.set_title("Movements of the robot inside the arena (Generation %d)" % generation)
    ax.legend(loc='best')
    plt.ioff()
    plt.show()


def observer(population, num_generations, num_evaluations, args):
    walls = args["walls"]
    startX = args["startX"]
    startY = args["startY"]
    startDegrees = args["startDegrees"]
    objectiveX = args["objectiveX"]
    objectiveY = args["objectiveY"]
    arena = args["arena"]

    max_evaluations = args.get('max_evaluations', 10000)

    if num_generations % 20 == 0:  # tous les 10%
        plot_generation(population, walls, arena, [startX, startY, startDegrees], [objectiveX, objectiveY],
                        generation=num_generations)
