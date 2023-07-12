import numpy as np
import seaborn as sns
import matplotlib.pylab as plt

for type in ['horizontal', 'vertical']:
    plt.figure()
    data = np.loadtxt(f'plot/{type}.txt', dtype=int)
    data = np.flip(data, axis=0)
    sns.heatmap(data, cmap='Spectral_r', vmin=-70, vmax=10)
    plt.gca().set_aspect('equal')
    plt.gca().invert_yaxis()
    plt.savefig(f'plot/{type}.png')
