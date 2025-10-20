import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform

class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

def quat_to_rotation_matrix(w, x, y, z):
    """Convert quaternion to rotation matrix"""
    R = np.array([
        [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
    ])
    return R

def draw_spacecraft(ax, pos, quat, scale=5000):
    """Draw spacecraft as coordinate axes showing orientation"""
    w, x, y, z = quat
    R = quat_to_rotation_matrix(w, x, y, z)
    
    # Body frame axes
    x_body = R @ np.array([1, 0, 0]) * scale
    y_body = R @ np.array([0, 1, 0]) * scale
    z_body = R @ np.array([0, 0, 1]) * scale
    
    arrows = []
    arrows.append(Arrow3D(pos[0], pos[1], pos[2], 
                         x_body[0], x_body[1], x_body[2],
                         mutation_scale=20, lw=2, arrowstyle="-|>", color="r"))
    arrows.append(Arrow3D(pos[0], pos[1], pos[2],
                         y_body[0], y_body[1], y_body[2],
                         mutation_scale=20, lw=2, arrowstyle="-|>", color="g"))
    arrows.append(Arrow3D(pos[0], pos[1], pos[2],
                         z_body[0], z_body[1], z_body[2],
                         mutation_scale=20, lw=2, arrowstyle="-|>", color="b"))
    
    for arrow in arrows:
        ax.add_artist(arrow)
    
    return arrows

# Load simulation data
simData = pd.read_csv('simulation_data.csv')

time = simData['time'].values
posX = simData['posX'].values
posY = simData['posY'].values
posZ = simData['posZ'].values
velX = simData['velX'].values
velY = simData['velY'].values
velZ = simData['velZ'].values
quatW = simData['quatW'].values
quatX = simData['quatX'].values
quatY = simData['quatY'].values
quatZ = simData['quatZ'].values

# Detect celestial body radius from data (distance from origin at landing)
# Assume last point is on the surface
BODY_RADIUS = np.sqrt(posX[-1]**2 + posY[-1]**2 + posZ[-1]**2)
print(f"Detected body radius: {BODY_RADIUS/1000:.1f} km")

# Determine body name based on radius
if abs(BODY_RADIUS - 6371000) < 100000:
    body_name = "Earth"
elif abs(BODY_RADIUS - 1737400) < 50000:
    body_name = "Moon"
elif abs(BODY_RADIUS - 3389500) < 100000:
    body_name = "Mars"
else:
    body_name = "Unknown Body"

print(f"Celestial body: {body_name}")

# Calculate altitude
altitude = np.sqrt(posX**2 + posY**2 + posZ**2) - BODY_RADIUS
speed = np.sqrt(velX**2 + velY**2 + velZ**2)

# === Plot 1: Altitude vs Time ===
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time, altitude / 1000, 'b-', linewidth=2)
ax1.set_xlabel('Time (s)', fontsize=12)
ax1.set_ylabel('Altitude (km)', fontsize=12)
ax1.set_title('Altitude vs Time', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)

ax2.plot(time, speed, 'r-', linewidth=2)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Speed (m/s)', fontsize=12)
ax2.set_title('Speed vs Time', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# === Plot 2: 3D Trajectory with Planet ===
fig2 = plt.figure(figsize=(12, 10))
ax = fig2.add_subplot(111, projection='3d')

# Draw celestial body sphere
u = np.linspace(0, 2 * np.pi, 50)
v = np.linspace(0, np.pi, 50)
x_sphere = BODY_RADIUS * np.outer(np.cos(u), np.sin(v))
y_sphere = BODY_RADIUS * np.outer(np.sin(u), np.sin(v))
z_sphere = BODY_RADIUS * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x_sphere, y_sphere, z_sphere, color='lightblue', alpha=0.3, edgecolor='none')

# Plot trajectory
ax.plot(posX, posY, posZ, 'r-', linewidth=2, label='Trajectory')

# Mark start and end points
ax.scatter([posX[0]], [posY[0]], [posZ[0]], color='green', s=100, label='Start', marker='o')
ax.scatter([posX[-1]], [posY[-1]], [posZ[-1]], color='red', s=100, label='Landing', marker='X')

# Draw spacecraft orientation at key points
num_frames = min(10, len(time))
indices = np.linspace(0, len(time)-1, num_frames, dtype=int)
for idx in indices:
    pos = np.array([posX[idx], posY[idx], posZ[idx]])
    quat = np.array([quatW[idx], quatX[idx], quatY[idx], quatZ[idx]])
    draw_spacecraft(ax, pos, quat, scale=50000)

ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_zlabel('Z (m)', fontsize=12)
ax.set_title(f'Spacecraft Landing Trajectory on {body_name}', fontsize=14, fontweight='bold')
ax.legend()

# Equal aspect ratio
max_range = np.array([posX.max()-posX.min(), posY.max()-posY.min(), posZ.max()-posZ.min()]).max() / 2.0
mid_x = (posX.max()+posX.min()) * 0.5
mid_y = (posY.max()+posY.min()) * 0.5
mid_z = (posZ.max()+posZ.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.tight_layout()
plt.show()

# === Plot 3: Velocity Components ===
fig3, axes = plt.subplots(3, 1, figsize=(10, 10))

axes[0].plot(time, velX, 'r-', linewidth=2)
axes[0].set_ylabel('Vx (m/s)', fontsize=12)
axes[0].set_title('Velocity Components', fontsize=14, fontweight='bold')
axes[0].grid(True, alpha=0.3)

axes[1].plot(time, velY, 'g-', linewidth=2)
axes[1].set_ylabel('Vy (m/s)', fontsize=12)
axes[1].grid(True, alpha=0.3)

axes[2].plot(time, velZ, 'b-', linewidth=2)
axes[2].set_ylabel('Vz (m/s)', fontsize=12)
axes[2].set_xlabel('Time (s)', fontsize=12)
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print(f"\n=== Landing Summary ===")
print(f"Initial altitude: {altitude[0]/1000:.2f} km")
print(f"Initial speed: {speed[0]:.2f} m/s")
print(f"Landing time: {time[-1]:.2f} s")
print(f"Final speed: {speed[-1]:.2f} m/s")
print(f"Landing position: ({posX[-1]/1000:.2f}, {posY[-1]/1000:.2f}, {posZ[-1]/1000:.2f}) km")
