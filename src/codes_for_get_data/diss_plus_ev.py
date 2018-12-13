#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

data0 = np.load("/home/cristinaescribano/CrazyDrones/src/multi_crazyflie_simulator/simulation_data/diss_plus_ev/crazyflie_0_data.npy")
time0 = np.load("/home/cristinaescribano/CrazyDrones/src/multi_crazyflie_simulator/simulation_data/diss_plus_ev/crazyflie_0_time.npy")
#data1 = np.load("/home/cristinaescribano/CrazyDrones/src/multi_crazyflie_simulator/simulation_data/crazyflie_1_data.npy")
#time1 = np.load("/home/cristinaescribano/CrazyDrones/src/multi_crazyflie_simulator/simulation_data/crazyflie_1_time.npy")

vector_mod = data0.reshape((np.shape(data0)[1], np.shape(data0)[2])).transpose()
#print(np.shape(vector_mod))
#print(np.squeeze(data0))

plt.plot( time0, vector_mod[0][:], color='g')
plt.plot( time0, vector_mod[1], color='orange')
plt.plot( time0, vector_mod[2], color='red')

plt.xlabel('time')
plt.ylabel('data')
plt.title('diss + e_v')
plt.show()