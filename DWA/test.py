import numpy as np
# a = np.array([1, 2, 3])
# b = np.ones(3)
# c = 0.5 * np.ones(2)
# d = np.ones((3, 2, 2))
# d[0] += 0.5
# d[:, 1] -= 0.5 
# print(d)
# print(d[:, :, 0])
a = []
a.append(np.ones(3))
a.append(np.arange(3))
print(a)
a = np.array(a)
print(a)