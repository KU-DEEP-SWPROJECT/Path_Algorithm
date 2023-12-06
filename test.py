import test_algorithm as t
import numpy as np

L = []
for i in range(4):
    x,y = map(int,input().split())
    L.append((x,y))
R = []
for i in range(4):
    x,y = map(int,input().split())
    R.append((x,y))

t.test_algorithm(200,15,np.array(L),np.array(R))


