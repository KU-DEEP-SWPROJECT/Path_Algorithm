import test_algorithm
import numpy as np
from robot_class import robot as Robot

robot = np.array( [(150, 31) ,(25, 26), (66, 27), (113, 32)])
SIZE = 200
RADIUS = 10
target = np.array([(81,160),(119,161),(120,119),(81,121)])
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)
