#!/usr/bin/python

import matplotlib.pyplot as plt
import seaborn as sns
# y_L = []
# sensor = "LASER"
# fname = "NIS"
# with open(fname+".txt") as csv:
#     nums = csv.read().split('\n')
#     for n in nums:
#         if n: y_L.append(float(n[len("LIDAR NIS: ")]))

# plt.style.use(['seaborn-whitegrid'])
# plt.figure(num=0, figsize=(12,4))
# plt.axhline(y=6.0, label="95%")
# plt.plot(y_L, '-', color='orange', label="NIS")
# plt.title(sensor)
# plt.xlabel("time step k")
# plt.ylabel("NIS")
# plt.legend()
# plt.savefig(sensor+".png")
# plt.show()


# y_R = []
# sensor = "RADAR"
# with open(fname+".txt") as csv:
#     nums = csv.read().split('\n')
#     for n in nums:
#         if n: y_R.append(float(n[len("RADAR NIS: ")]))

# plt.style.use(['seaborn-whitegrid'])
# plt.figure(num=1, figsize=(12,4))
# plt.axhline(y=7.8, label="95%")
# plt.plot(y_R, '-', color='orange', label="NIS")
# plt.title(sensor)
# plt.xlabel("time step k")
# plt.ylabel("NIS")
# plt.legend()
# plt.savefig(sensor+".png")
# plt.show()


import numpy as np  
import pandas as pd  
  
metrics = ['X', 'Y', 'Vx', 'Vy']
# load the dataset 
df1 = pd.read_csv("lidar_radar.csv", names=metrics, index_col=False).assign(Sensor='lidar+radar')
df2 = pd.read_csv("lidar_only_acc2.csv", names=metrics, index_col=False).assign(Sensor='lidar')
df3 = pd.read_csv("radar_only_acc2.csv", names=metrics, index_col=False).assign(Sensor='radar')
cdf = pd.concat([df1, df2, df3])    
mdf = pd.melt(cdf,id_vars=["Sensor"], var_name='metrics', value_name='RMSE')
print(mdf['metrics'])
sns.boxplot(x="Sensor", y='RMSE', hue='metrics', data=mdf)
plt.savefig("accuracy_with_outlier"+".png")
plt.show()



