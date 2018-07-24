import math
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# %matplotlib inline

# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display

plt.ion()

def plot_durations(y):
    plt.figure(2)
    plt.clf()
    plt.subplot(211)
    plt.plot(y[:,0])
    plt.subplot(212)
    plt.plot(y[:,1])

    plt.pause(0.001)  # pause a bit so that plots are updated
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())



x = np.linspace(-10,10,500)
y = []
for i in range(len(x)):
    y1 = np.cos(i/(3*3.14))
    y2 = np.sin(i/(3*3.14))
    y.append(np.array([y1,y2]))
    plot_durations(np.array(y))