#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

params = {'backend': 'ps',
          'axes.labelsize': 20,
          'axes.unicode_minus': False,
          'text.fontsize': 20,
          'legend.fontsize': 20,
          'xtick.labelsize': 20,
          'ytick.labelsize': 20,
          "figure.subplot.right": 0.96,
          "figure.subplot.top": 0.96,
          "figure.subplot.left": 0.12,
          "figure.subplot.bottom": 0.15,
          'figure.figsize': [7, 4.5],
          'ps.useafm': True,
          'pdf.use14corefonts': True,
          'text.usetex': True
          }
plt.rcParams.update(params)

w = 0.04

# Y1 = np.array([3,1,4])
# Y2 = np.array([1,5,9])
# X = np.array([1,2,3])
import plot_dat as pd

plt.bar(pd.X - w/2, pd.Y1, color='b', width=w, label="Torque Gradient", align="center")
plt.bar(pd.X + w/2, pd.Y2, color='g', width=w, label="Qseudo Gradient", align="center")

plt.legend(loc="best")
plt.ylabel("histgram")
plt.xlabel("$\|Optimal Torque\|$/$\|Intial Torque\|$")

plt.xticks(pd.X, pd.X)

plt.show()
