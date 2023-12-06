import test_algorithm
import numpy as np
from robot_class import robot as Robot

robot = np.array( [(154, 24) ,(31, 27), (70, 24), (116, 25)])
SIZE = 200
RADIUS = 5
target = np.array([(110,110),(150,110),(150,70),(110,70)])
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)

