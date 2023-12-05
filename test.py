import test_algorithm
import numpy as np
from robot_class import robot as Robot

robot = np.array( [(41, 12) ,(56, 11), (26, 12), (13, 11)])
SIZE = 100
RADIUS = 10
target = np.array([(10,77),(29,76),(29,56),(10,57)])
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)
