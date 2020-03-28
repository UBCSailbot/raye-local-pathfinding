import math
import matplotlib.pyplot as plt
from utilities import * 
from updated_geometric_planner import Obstacle
import numpy

obstacle = Obstacle(0, 0, 5, 2, 45)
plt.xlim(-10, 10)
plt.ylim(-10, 10)
ax = plt.gca()
plt.plot([obstacle.x], [obstacle.y], marker='o', markersize=1, color="orange")
for i in range(0, 180):
#    point = ellipseFormula(obstacle, i)
#    point = list(point)
    point = ellipseFormula2(obstacle, i)
    plt.plot([point[0]], [point[1]], marker = 'o', markersize= 1, color="red")
ax.add_patch(patches.Ellipse((obstacle.x, obstacle.y), obstacle.width, obstacle.height, obstacle.angle))
plt.show()
