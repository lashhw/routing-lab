#The code was jointly developed by members of VDALab and EDALab.
import numpy as np
import random
import matplotlib.pyplot as plt
import argparse

def int_to_color(net_id):
    color_str = "#"
    random.seed(net_id)
    for _ in range(6):
        number = random.randint(1, 14)
        if number == 14:
            color_str += "E"
        elif number == 13:
            color_str += "D"
        elif number == 12:
            color_str += "C"
        elif number == 11:
            color_str += "B"
        elif number == 10:
            color_str += "A"
        else:
            color_str += str(number)

    return color_str

def draw_line(plt, x1 ,y1 ,x2 ,y2 ,color):
    if x1 == x2:
        new_x1,new_x2 = x1,x2
    else:
        new_x1,new_x2 = x1,x2
    if y1 == y2:
        new_y1,new_y2 = y1,y2
    else:
        new_y1,new_y2 = y1,y2
    plt.plot([new_x1, new_x2], [new_y1,new_y2], color=color, ls="-", linewidth=2)

def draw_img(top_row, bottom_row, nets_segment, maze_x_range, maze_y_range, title, filename):
    grid_length = 1 
    fig_max_scale = max(6, int(max(maze_x_range[1],maze_y_range[1]) / 10) + 1)
    fig = plt.figure(figsize=(fig_max_scale,fig_max_scale))
    #fig = plt.figure(figsize=(6,6))
    fig.add_subplot(111)

    plt.axis([maze_x_range[0], maze_x_range[1]+1, maze_y_range[0], maze_y_range[1] + 1])
    plt.axis("on")
    plt.grid(True,ls="--")
    plt.title(title)
    plt.xticks(np.arange(0,maze_x_range[1] + 1, grid_length), fontsize=3)
    plt.yticks(np.arange(0,maze_y_range[1] + 1, grid_length), fontsize=3)
    plt.xlim(xmin=maze_x_range[0]-0.5, xmax=maze_x_range[1]+0.5)
    plt.ylim(ymin=maze_y_range[0]-1, ymax=maze_y_range[1]+1)

    for i in range(len(top_row)):
        now_color = "#666666"
        if top_row[i] != 0:
            now_color = int_to_color(top_row[i])
        plt.text(i, maze_y_range[1] + 0.5, str(top_row[i]), fontstyle='oblique',backgroundcolor=now_color,color="#FFFFFF", horizontalalignment='center', verticalalignment='center', fontsize = 3)
        now_color = "#666666"
        if bottom_row[i] != 0:
            now_color = int_to_color(bottom_row[i])
        plt.text(i, -0.5 , str(bottom_row[i]), fontstyle='oblique',backgroundcolor=now_color,color="#FFFFFF", horizontalalignment='center', verticalalignment='center', fontsize = 3)
    
    for net_name, segments in nets_segment.items():
        net_color = int_to_color(int(net_name))
        for segment in segments:
            draw_line(plt,segment[0],segment[1],segment[2],segment[3],net_color)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig(filename, dpi=300)
    # plt.show()

if __name__== "__main__" :
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file",   type=str,  help="input file(testcase) filename")
    parser.add_argument("--out_file",  type=str,  help="output file(routing result) filename")
    parser.add_argument("--img_name",  type=str,  help="image filename")
    args = parser.parse_args()

    top_row = []
    bottom_row = []
    nets_segment = {}

    with open(args.in_file, 'r') as f:
        lines = f.readlines()
        top_row = [int(x) for x in lines[0].split()]
        bottom_row = [int(x) for x in lines[1].split()]
    min_bound_x = 0
    max_bound_x = max(1, len(top_row) - 1)
    min_bound_y = 0
    max_bound_y = 1
    with open(args.out_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.split()
            if line[0] == ".begin":
                net_name = line[1]
                nets_segment[net_name] = []
            elif line[0] == ".end":
                pass
            else:
                assert len(line) >= 4, "Wrong output format"
                r1 =  int(line[1])
                r2 =  int(line[2])
                r3 =  int(line[3])
                if line[0] == ".V":
                    max_bound_y = max(max_bound_y, r2, r3)
                    nets_segment[net_name].append([r1,r2,r1,r3])
                elif line[0] == ".H":
                    max_bound_x = max(max_bound_x, r1, r3)
                    min_bound_x = min(min_bound_x, r1, r3)
                    nets_segment[net_name].append([r1,r2,r3,r2])
                else:
                    raise ValueError("Wrong output format")
    draw_img(top_row, bottom_row, nets_segment, (min_bound_x, max_bound_x), (min_bound_y, max_bound_y), args.in_file[:-4], args.img_name)





