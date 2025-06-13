import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

a = np.zeros((5, 5))
a[2, 2] = 1

WIDTH=120
HEIGHT=120

df = pd.read_csv("data/nm2/_map_convert.csv")
for i,msg in enumerate(df["data"]):
    index = msg.find("data=")
    array_start = index+6
    occupancy_str = msg[array_start:-2]
    values = [int(i) for i in occupancy_str.split(',')]
    # bin_values = (values == 100)

    occupancy_arr = np.array(values).reshape(HEIGHT,WIDTH)
    for index,value in enumerate(occupancy_arr):
        if value !=0:
            
    # plt.imshow(occupancy_arr, cmap='gray', interpolation='none', origin='lower')
    # plt.show()
    buffered_binary = ndimage.binary_dilation(occupancy_arr,iterations=2).astype(occupancy_arr.dtype)
    plt.imshow(buffered_binary, cmap='gray', interpolation='none', origin='lower')
    plt.axis('off')

    # plt.axis('off')
    # plt.show()
    fname = f"data/nm2/figures/msg{i}.png"
    plt.savefig(fname, bbox_inches='tight', pad_inches=0) 
