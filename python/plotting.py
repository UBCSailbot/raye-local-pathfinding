import numpy as np
import matplotlib.pyplot as plt


def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

def plot_path(solution_path, dimensions, obstacles):
    matrix = solution_path.printAsMatrix()
    path = create_numpy_path(matrix)
    x, y = path.T
    ax = plt.gca()
    ax.plot(x, y, 'r--')
    ax.plot(x, y, 'go') 
    ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
    for obstacle in obstacles:
        ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))

    plt.show()
