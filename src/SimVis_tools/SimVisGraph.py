# SimVisGraph
# Used for outputting trajectory data on graphs

import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

simData = pd.read_csv('simulation_data.csv') # time,posX,posY,posZ,velX,velY,velZ,quatW,quatX,quatY,quatZ,omegX,omegY,omegZ

time = simData['time'].tolist()

posX = simData['posX'].tolist()
posY = simData['posY'].tolist()
posZ = simData['posZ'].tolist()

velX = simData['velX'].tolist()
velY = simData['velY'].tolist()
velZ = simData['velZ'].tolist()

quatW = simData['quatW'].tolist()
quatX = simData['quatX'].tolist()
quatY = simData['quatY'].tolist()
quatZ = simData['quatZ'].tolist()

omegX = simData['omegX'].tolist()
omegY = simData['omegY'].tolist()
omegZ = simData['omegZ'].tolist()

roll = simData['roll'].tolist()
pitch = simData['pitch'].tolist()
yaw = simData['yaw'].tolist()

plt.plot(time, roll)
plt.xlabel('Time (s)')
plt.ylabel('Z Position (m)')
plt.title('Z Position vs Time')
plt.grid(True)
plt.show() 

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(posX, posY, posZ, label='3D trajectory')
#ax.scatter(posX, posY, posZ, c=time, cmap='viridis')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Spacecraft Trajectory')

ax.legend()
plt.show() 




