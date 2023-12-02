import numpy as np
a = np.array([[0,0],[4,4],[4,0],[0,4]])
t = tuple(np.mean(a,axis=0).astype(int))
print(t)