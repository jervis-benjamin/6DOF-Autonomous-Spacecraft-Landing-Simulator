# SimVisOrientation
# 3D orientation visualization of spacecraft

import numpy as np
import pandas as pd
import time
import trimesh
from vispy import scene, app
from vispy.scene.visuals import Text, Line
from vispy.visuals.transforms import MatrixTransform
from scipy.spatial.transform import Rotation\


# === Helper functions ==
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) / 255.0 for i in (0, 2, 4))

# === Load quaternion data ===
df = pd.read_csv("simulation_data.csv")
times = df["time"].values
pos = df[["posX", "posY", "posZ"]].values
quats = df[["quatW", "quatX", "quatY", "quatZ"]].values
n_frames = len(quats)
data_timestep = times[1] - times[0] if len(times) > 1 else 0.01

# === Load STL ===
mesh = trimesh.load_mesh("my_spacecraft.stl")

# --- Center and scale the mesh ---
mesh.vertices -= mesh.centroid
scale = 1.0 / np.max(mesh.extents)
mesh.vertices *= scale

# --- Manual translation offset (adjust rotation pivot) ---
manual_offset = np.array([0.0, 0.0, 0.0])

# === Apply calibration rotation to MESH ONLY ===
spacecraft_calibrate_rot = (
    trimesh.transformations.rotation_matrix(np.radians(90), [0, 1, 0])
)

mesh.apply_transform(spacecraft_calibrate_rot)

# === Separate calibration for AXES ===
# Adjust these angles to orient your axes independently
axes_calibrate_rot = (
    trimesh.transformations.rotation_matrix(np.radians(0), [1, 0, 0]) @
    trimesh.transformations.rotation_matrix(np.radians(90), [0, 1, 0]) @
    trimesh.transformations.rotation_matrix(np.radians(0), [0, 0, 1])
)
axes_calibrate_rot_3x3 = axes_calibrate_rot[:3, :3]

# === Setup Vispy Scene ===
canvas = scene.SceneCanvas(keys='interactive', show=True, bgcolor='black', size=(900, 700))
view = canvas.central_widget.add_view()
view.camera = scene.TurntableCamera(up='z', fov=45, distance=3)

# === Spacecraft Mesh ===
spacecraft = scene.visuals.Mesh(
    vertices=mesh.vertices,
    faces=mesh.faces,
    color=(0.7, 0.7, 1.0, 1.0),
    shading='smooth',
    parent=view.scene
)

# === Body Axes (X-red, Y-green, Z-blue) ===
axis_length = 0.8
# Define the original axis directions (DON'T apply calibration here)
axis_dirs_original = [
    np.array([[0, 0, 0], [axis_length, 0, 0]]),  # +X
    np.array([[0, 0, 0], [0, axis_length, 0]]),  # +Y
    np.array([[0, 0, 0], [0, 0, axis_length]]),  # +Z
]

# Create axis lines WITHOUT pre-applying calibration
axis_lines = [
    Line(axis_dirs_original[0], color='red', width=3, parent=view.scene),    # +X
    Line(axis_dirs_original[1], color='green', width=3, parent=view.scene),  # +Y
    Line(axis_dirs_original[2], color='blue', width=3, parent=view.scene),   # +Z
]

# === World Axes (static, fixed to world frame) ===
world_axis_length = 0.5
world_axis_offset = np.array([-1.0, -1.0, -0.7])  # Position in bottom-left area of scene

world_axis_dirs = [
    np.array([world_axis_offset, world_axis_offset + [world_axis_length, 0, 0]]),  # +X
    np.array([world_axis_offset, world_axis_offset + [0, world_axis_length, 0]]),  # +Y
    np.array([world_axis_offset, world_axis_offset + [0, 0, world_axis_length]]),  # +Z
]

world_axis_lines = [
    Line(world_axis_dirs[0], color=hex_to_rgb('#f25060'), width=2, parent=view.scene),    # +X
    Line(world_axis_dirs[1], color=hex_to_rgb('#50f28b'), width=2, parent=view.scene),  # +Y
    Line(world_axis_dirs[2], color=hex_to_rgb('#50c4f2'), width=2, parent=view.scene),   # +Z
]

# Add labels for world axes
world_x_text = Text('world x', parent=canvas.scene, color=hex_to_rgb('#f25060'), bold=True, font_size=10, face='Tahoma')
world_y_text = Text('world y', parent=canvas.scene, color=hex_to_rgb('#50f28b'), bold=True, font_size=10, face='Tahoma')
world_z_text = Text('world z', parent=canvas.scene, color=hex_to_rgb('#50c4f2'), bold=True, font_size=10, face='Tahoma')

# === Text ===
time_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=16, face='Tahoma')
time_text.pos = 155, 35

pos_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=7, face='Tahoma')
pos_text.anchors = ('right', 'top')

x_axis_text = Text('body x', parent=canvas.scene, color='red', bold=True, font_size=10, face='Tahoma')
y_axis_text = Text('body y', parent=canvas.scene, color='green', bold=True, font_size=10, face='Tahoma')
z_axis_text = Text('body z', parent=canvas.scene, color='blue', bold=True, font_size=10, face='Tahoma')

# === Separate Transformations ===
spacecraft_transform = MatrixTransform()
axes_transform = MatrixTransform()

spacecraft.transform = spacecraft_transform
for axis in axis_lines:
    axis.transform = axes_transform

# Function to update text positions based on canvas size
def update_text_positions():

    w, h = canvas.size

    pos_text.pos = w - 20, 35

    # Position legend at bottom right with offset from edges
    x_axis_text.pos = w - 70, h - 100
    y_axis_text.pos = w - 70, h - 60
    z_axis_text.pos = w - 70, h - 20
    
    # Position world axes labels at bottom left
    world_x_text.pos = 70, h - 100
    world_y_text.pos = 70, h - 60
    world_z_text.pos = 70, h - 20

# Connect the resize event
@canvas.events.resize.connect
def on_resize(event):
    update_text_positions()

# Set initial positions
update_text_positions()

# === Animation update ===
index = 0
start_time = time.perf_counter()

def update(event):
    global index, start_time

    elapsed = time.perf_counter() - start_time
    target_index = int(elapsed / data_timestep)
    if target_index >= n_frames:
        start_time = time.perf_counter()
        target_index = 0
    index = target_index

    q = quats[index]
    position = pos[index]

    # Convert to rotation matrix
    r = Rotation.from_quat([q[1], q[2], q[3], -q[0]])

    # Get the rotation matrix
    R = r.as_matrix()

    # Create the animation rotation matrix
    rot_mat = np.eye(4)
    rot_mat[:3, :3] = R
    rot_mat[:3, 3] = manual_offset
    
    # For spacecraft: compose the animation rotation WITH the axes calibration rotation
    # Apply the same rotation composition so spacecraft rotates with the axes
    spacecraft_combined = np.eye(4)
    spacecraft_combined[:3, :3] = R @ axes_calibrate_rot_3x3
    spacecraft_combined[:3, 3] = manual_offset
    spacecraft_transform.matrix = spacecraft_combined
    
    # For axes: compose the animation rotation WITH the calibration rotation
    axes_combined = np.eye(4)
    axes_combined[:3, :3] = R @ axes_calibrate_rot_3x3
    axes_combined[:3, 3] = manual_offset
    axes_transform.matrix = axes_combined

    time_text.text = f"Time: {times[index]:.2f} s"

    pos_text.text = f"pos(x y z): [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}] m"

timer = app.Timer(interval=1/60, connect=update, start=True)

if __name__ == '__main__':
    print("Running with timestep:", data_timestep)
    app.run()