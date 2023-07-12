import matplotlib.pyplot as plt
import numpy as np

rng = np.random.default_rng(seed=0)

with open("plot/net.txt") as f:
    num_nets = int(f.readline())
    for net_id in range(num_nets):
        p = rng.choice([True, False], p=[0.01, 0.99])
        net_name, num_pins, num_hpath, num_vpath = f.readline().split()
        num_pins = int(num_pins)
        num_hpath = int(num_hpath)
        num_vpath = int(num_vpath)
        for _ in range(num_pins):
            x, y = [int(x) for x in f.readline().split()]
            if p:
                plt.plot(x, y, 'ro')
        for _ in range(num_hpath):
            x, y = [int(x) for x in f.readline().split()]
            if p:
                plt.plot([x, x + 1], [y, y], 'b')
        for _ in range(num_vpath):
            x, y = [int(x) for x in f.readline().split()]
            if p:
                plt.plot([x, x], [y, y + 1], 'b')
        if p:
            plt.gca().set_aspect('equal')
            plt.savefig(f'plot/net/{net_name}.png')
            plt.clf()
