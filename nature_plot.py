from matplotlib import pyplot as plt
import numpy as np


t = np.linspace(0, 1, 1000)
y = 1 / np.e**(t/0.2)
re_y = np.flip(y)
plt.plot(re_y)
plt.show()