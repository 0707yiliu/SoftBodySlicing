from matplotlib import pyplot as plt
import numpy as np


t = np.linspace(0, 0.1, 1000)
y = np.e**(-t/0.01)
re_y = np.flip(y)
plt.plot(y)
plt.show()