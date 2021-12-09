import numpy as np
a = np.array([1, 2, 3])
b = np.ones(3)
c = np.c_[a, b]
print(np.max(c[1]))