import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import matplotlib.patches as mpatches

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
cmap = plt.cm.tab20

patches = []

with open(sys.argv[1]) as f:
    num_nets = int(f.readline())
    norm = Normalize(vmin=0, vmax=num_nets - 1)
    for i in range(num_nets):
        patches.append(mpatches.Patch(color=cmap(norm(i)), label=str(i)))

    num_h = int(f.readline())
    for _ in range(num_h):
        x, y, layer, net = map(int, f.readline().split())
        ax.plot([x, x + 1], [y, y], [layer, layer], c=cmap(norm(net)))
    
    num_v = int(f.readline())
    for _ in range(num_v):
        x, y, layer, net = map(int, f.readline().split())
        ax.plot([x, x], [y, y + 1], [layer, layer], c=cmap(norm(net)))
        
    num_layer = int(f.readline())
    for _ in range(num_layer):
        x, y, layer, net = map(int, f.readline().split())
        ax.plot([x, x], [y, y], [layer, layer + 1], c=cmap(norm(net)))

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.legend(handles=patches)
fig.tight_layout()
plt.show()
