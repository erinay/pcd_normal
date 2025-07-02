import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

WIDTH=120
HEIGHT=120
RESOLUTION=0.05

def inverse_model(map):
    # No buffering c]
    lhit = prob2likelihood(0.9)
    lmiss = prob2likelihood(0.45)
    Lk = np.zeros((HEIGHT,WIDTH))
    for i in range(HEIGHT):
        for j in range(WIDTH):
            if map[i][j] == 100:
                Lk[i][j] = lhit
            else:
                Lk[i][j] = lmiss
    return Lk
def prob2likelihood(map):
    log_map = np.log(map/(1-map))
    return log_map
def likelihood2prob(map):
    prob_map = 1-1/(1+np.exp(map))
    return prob_map

map = np.ones([120,120])*0.5
likelihood_map = prob2likelihood(map)
lt = likelihood_map.copy()
## NOTES: Start occupancy mapping with no motion (no transform info)
i=0
df = pd.read_csv("data/CorrectFilter/_map_convert.csv")
for i,msg in enumerate(df["data"]):
    ##IGNORE Ocuupancy MAPPING>>>>
    # first shadow points by 0.5 m
    index = msg.find("data=")
    array_start = index+6
    occupancy_str = msg[array_start:-2]
    values = [int(i) for i in occupancy_str.split(',')]
    occupancy_arr = np.array(values).reshape(HEIGHT,WIDTH)

    
    # # l_t = l_t-1+inverse_model
    # inv_map = inverse_model(occupancy_arr)
    # lt += inv_map-likelihood_map
    # # lt = max(min(lt, 0.99), 0.12)
    # map_t = likelihood2prob(lt)*255
    # # print(map_t)
    # plt.imshow(map_t, cmap='grey', interpolation='none', origin='lower')
    # plt.axis('off')
    # fname = f"data/CorrectFilter/occup_figures/msg{i}.png"
    # plt.savefig(fname, bbox_inches='tight', pad_inches=0)
    # i+=1
