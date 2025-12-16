import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

df = pd.read_csv("rk_demo/data/valid_workspace_rounded.csv")


X = df['X'] # First column data
Y = df['Y'] # Second column data
Z = df['Z'] # Third column data

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(X, Y, Z, c=Z, s=0.05 )

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_ylabel("Z [m]")

plt.savefig("rk_demo/figures/valid_workspace_rounded.png", dpi=1200)