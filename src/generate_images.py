import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

WIDTH=120
HEIGHT=120

df = pd.read_csv("data/NoMotionF/_map_convert.csv")
for i,msg in enumerate(df["data"]):
    index = msg.find("data=")
    array_start = index+6
    occupancy_str = msg[array_start:-2]
    values = [int(i) for i in occupancy_str.split(',')]
    occupancy_arr = np.array(values).reshape(HEIGHT,WIDTH)
    plt.imshow(occupancy_arr, cmap='gray', interpolation='none', origin='lower')
    plt.axis('off')
    fname = f"data/NoMotionF/figures/msg{i}.png"
    plt.savefig(fname, bbox_inches='tight', pad_inches=0) 
