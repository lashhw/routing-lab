import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 讀取繞線txt檔案
with open(sys.argv[1], 'r') as f:
    lines = f.readlines()[4:]

# 解析繞線
paths = {}
current_path = None
for line in lines:
    line = line.strip()
    if line.startswith('net') or line.startswith( 'Net' ):
        current_path = []
        paths[line] = current_path
    else:
        x, y, z = map(int, line.split())
        current_path.append((x, y, z))

# 繪製3D圖
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for name, path in paths.items():
    xs, ys, zs = zip(*path)
    ax.plot(xs, ys, zs, label=name, marker='o', markevery=[0, -1])

# 設置坐標軸標籤和範圍
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_zlim(0, None)

# 設置圖例字型大小，並縮小圖例
plt.legend(fontsize=4)
fig.tight_layout()

# 設置刻度字型大小
ax.tick_params(axis='both', which='minor', labelsize=16)

# 儲存和顯示圖片
plt.savefig("path.png")
plt.show()
