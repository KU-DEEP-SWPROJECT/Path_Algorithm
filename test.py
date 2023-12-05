import test_algorithm
import numpy as np
from robot_class import robot as Robot

robot = np.array( [(154, 24) ,(31, 27), (70, 24), (116, 25)])
SIZE = 200
RADIUS = 10
target = np.array([(84,160),(123,158),(121,118),(82,121)])
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)
