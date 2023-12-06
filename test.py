import test_algorithm
import numpy as np
from robot_class import robot as Robot

L = []
R = []
for i in range(4):
    L.append(tuple(map(int,input().split())))
for i in range(4):
    R.append(tuple(map(int,input().split())))


SIZE = 200
RADIUS = 10
robot = np.array(L)
target = np.array(R)
test_algorithm.test_algorithm(SIZE,RADIUS,robot,target)

