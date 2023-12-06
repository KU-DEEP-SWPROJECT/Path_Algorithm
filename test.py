import test_algorithm
import numpy as np
from robot_class import robot as Robot

robot = np.array( [(33, 26) ,(109, 26), (151, 27), (70, 25)])
SIZE = 200
RADIUS = 10
target = np.array([(80,163),(120,164),(119,122),(79,123)])
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)

