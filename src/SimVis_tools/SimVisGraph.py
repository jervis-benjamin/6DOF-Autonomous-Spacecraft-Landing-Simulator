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

fig, axs = plt.subplots(2, 3, figsize=(12, 6))

# Top row
axs[0, 0].plot(time, posX)
axs[0, 0].set_title('PosX vs Time')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('PosX (m)')

axs[0, 1].plot(time, posY)
axs[0, 1].set_title('PosY vs Time')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('PosY (m)')

axs[0, 2].plot(time, posZ)
axs[0, 2].set_title('PosZ vs Time')
axs[0, 2].set_xlabel('Time (s)')
axs[0, 2].set_ylabel('PosZ (m)')

# Bottom row
axs[1, 0].plot(time, roll)
axs[1, 0].set_title('Roll vs Time')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Roll (deg)')

axs[1, 1].plot(time, pitch)
axs[1, 1].set_title('Pitch vs Time')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Pitch (deg)')

axs[1, 2].plot(time, yaw)
axs[1, 2].set_title('Yaw vs Time')
axs[1, 2].set_xlabel('Time (s)')
axs[1, 2].set_ylabel('Yaw (deg)')

fig.tight_layout()
plt.show()

fig1, axs1 = plt.subplots(2, 3, figsize=(12, 6))
# Top row
axs1[0, 0].plot(time, velX)
axs1[0, 0].set_title('VelX vs Time')
axs1[0, 0].set_xlabel('Time (s)')
axs1[0, 0].set_ylabel('VelX (m/s)')

axs1[0, 1].plot(time, velY)
axs1[0, 1].set_title('VelY vs Time')
axs1[0, 1].set_xlabel('Time (s)')
axs1[0, 1].set_ylabel('VelY (m/s)')

axs1[0, 2].plot(time, velZ)
axs1[0, 2].set_title('VelZ vs Time')
axs1[0, 2].set_xlabel('Time (s)')
axs1[0, 2].set_ylabel('VelZ (m/s)')

# Bottom row
axs1[1, 0].plot(time, omegX)
axs1[1, 0].set_title('OmegaX vs Time')
axs1[1, 0].set_xlabel('Time (s)')
axs1[1, 0].set_ylabel('OmegaX (rad/s)')

axs1[1, 1].plot(time, omegY)
axs1[1, 1].set_title('OmegaY vs Time')
axs1[1, 1].set_xlabel('Time (s)')
axs1[1, 1].set_ylabel('OmegaY (rad/s)')

axs1[1, 2].plot(time, omegZ)
axs1[1, 2].set_title('OmegaZ vs Time')
axs1[1, 2].set_xlabel('Time (s)')
axs1[1, 2].set_ylabel('OmegaZ (rad/s)')

fig1.tight_layout()
plt.show()

fig2, axs2 = plt.subplots(2, 2, figsize=(12, 6))
# Top row
axs2[0, 0].plot(time, quatW)
axs2[0, 0].set_title('quatW vs Time')
axs2[0, 0].set_xlabel('Time (s)')
axs2[0, 0].set_ylabel('quatW')

axs2[0, 1].plot(time, quatX)
axs2[0, 1].set_title('quatX vs Time')
axs2[0, 1].set_xlabel('Time (s)')
axs2[0, 1].set_ylabel('quatX')

# Bottom row
axs2[1, 0].plot(time, quatY)
axs2[1, 0].set_title('quatY vs Time')
axs2[1, 0].set_xlabel('Time (s)')
axs2[1, 0].set_ylabel('quatY')

axs2[1, 1].plot(time, quatZ)
axs2[1, 1].set_title('quatZ vs Time')
axs2[1, 1].set_xlabel('Time (s)')
axs2[1, 1].set_ylabel('quatZ')
fig2.tight_layout()
plt.show()


fig3d = plt.figure()
ax = fig3d.add_subplot(111, projection='3d')

ax.plot(posX, posY, posZ, label='3D trajectory')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Spacecraft Trajectory')

ax.legend()
plt.show() 