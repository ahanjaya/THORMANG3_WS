#!/usr/bin/env python3

import math
import random
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def calc_dist(x):
    target_pos     = np.array([ [-1.5, 0.0] ])
    current_pos    = np.array([ [x, 0.0] ])
    euclidean_dist = np.linalg.norm(target_pos - current_pos, axis=1)
    return np.asscalar(euclidean_dist)


reward = interp1d([1.5,0], [0, 1])

for i in range(16):
    i = - ( i /10 )

    err = calc_dist(i)

    han = reward(err)
    reduced_reward = 0.3 * han

    print( 'err: {}, rewards: {}, dist: {}'.format(err, han, reduced_reward) )
