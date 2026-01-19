# SimVis3d
# 3D orientation and translation visualization of trajectory
# *Lunar Lander model from NASA
# **General architecture and certain VisPy functions suggested by Claude as learning animations is not a main objective of this project

import numpy as np
import pandas as pd
import trimesh
from vispy import scene, app
from vispy.scene.visuals import Text, Line
from vispy.visuals.transforms import MatrixTransform
from scipy.spatial.transform import Rotation as R

def create_cone_geometry(length=3.0, base_radius=1.0, resolution=32):
    # cone, which starts at the origin of the body frame, to visualize engine plume
    
    tip = np.array([0.0, 0.0, 0.0])
    
    angles = np.linspace(0, 2 * np.pi, resolution, endpoint=False)
    base_vertices = np.zeros((resolution, 3))
    base_vertices[:, 0] = -length  # Extend in -X direction (body frame)
    base_vertices[:, 1] = base_radius * np.cos(angles)  # Y
    base_vertices[:, 2] = base_radius * np.sin(angles)  # Z
    
    vertices = np.vstack([tip, base_vertices])
    
    faces = []
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([0, i + 1, next_i + 1])
    
    center_idx = len(vertices)
    base_center = np.array([[-length, 0.0, 0.0]])
    vertices = np.vstack([vertices, base_center])
    
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([center_idx, next_i + 1, i + 1])
    
    return vertices.astype(np.float32), np.array(faces, dtype=np.uint32)

def quat_to_rotation_matrix(q):
    # convert quaternion to rotation matrix
    w, x, y, z = q
    
    R_mat = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    return R_mat

# for customizing object colors
def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) / 255.0 for i in (0, 2, 4))

#df = pd.read_csv("simulation_data.csv")
df = pd.read_parquet("../data/simulation_data.parquet")
pos = df[["posX (m)", "posY (m)", "posZ (m)"]].values
vel = df[["velX (m/s)", "velY (m/s)", "velZ (m/s)"]].values

quat = df[["quatW", "quatX", "quatY", "quatZ"]].values
euler_ang = df[["pitch (deg)", "yaw (deg)", "roll (deg)"]].values

throttle = df["throttleLevel (%)"].values / 100
prop = df["propLevel (%)"].values

times = df["time (s)"].values

mesh = trimesh.load("lunar_lander.stl")

mesh.vertices -= mesh.centroid
scale = 1.0 / np.max(mesh.extents)
mesh.vertices *= scale

spacecraft_calibrate_rot = (
    trimesh.transformations.rotation_matrix(np.radians(90), [0, 1, 0]) @
    trimesh.transformations.rotation_matrix(np.radians(90), [0, 0, 1])
)
mesh.apply_transform(spacecraft_calibrate_rot)

axes_calibrate_rot = (
    trimesh.transformations.rotation_matrix(np.radians(0), [1, 0, 0]) @
    trimesh.transformations.rotation_matrix(np.radians(90), [0, 1, 0]) @
    trimesh.transformations.rotation_matrix(np.radians(0), [0, 0, 1])
)
axes_calibrate_rot_3x3 = axes_calibrate_rot[:3, :3]

canvas = scene.SceneCanvas(keys='interactive', bgcolor='black', size=(1200, 700), show=True)
view = canvas.central_widget.add_view()

axis_length = 0.65
axis_dirs_original = [
    np.array([[0, 0, 0], [axis_length, 0, 0]]),  # +X (red)
    np.array([[0, 0, 0], [0, axis_length, 0]]),  # +Y (green)
    np.array([[0, 0, 0], [0, 0, axis_length]]),  # +Z (blue)
]

axis_lines = [
    Line(axis_dirs_original[0], color='red', width=3, parent=view.scene),
    Line(axis_dirs_original[1], color='green', width=3, parent=view.scene),
    Line(axis_dirs_original[2], color='blue', width=3, parent=view.scene),
]
axes_transform = MatrixTransform()
for axis in axis_lines:
    axis.transform = axes_transform

ground_size = 10000000
ground = scene.visuals.Plane(width=ground_size, height=ground_size,
                             width_segments=20, height_segments=20,
                             parent=view.scene, color=hex_to_rgb("#255529"))
ground_transform = MatrixTransform()
ground.transform = ground_transform
ground.transform.translate((0, 0, 0))

grid = scene.visuals.GridLines(parent=view.scene, color=hex_to_rgb("#7A7A7A"))
grid.transform = MatrixTransform()
grid.transform.scale((ground_size/20, ground_size/20, 1))
grid.transform.translate((0, 0, 0.01))

num_stars = 1000
star_positions = np.random.randn(num_stars, 3)
star_positions = star_positions / np.linalg.norm(star_positions, axis=1, keepdims=True)  
star_positions = star_positions * np.random.uniform(2000, 3000, (num_stars, 1)) 

star_base_alpha = np.random.uniform(0.3, 1.0, num_stars) 
star_colors = np.ones((num_stars, 4))
star_colors[:, :3] = 1.0  
star_colors[:, 3] = star_base_alpha

starfield = scene.visuals.Markers(
    pos=star_positions,
    size=3,  
    edge_width=0,
    face_color=star_colors,
    parent=view.scene
)
starfield.visible = False 
starfield_transform = MatrixTransform()
starfield.transform = starfield_transform

spacecraft = scene.visuals.Mesh(
    vertices=mesh.vertices,
    faces=mesh.faces,
    color=(0.7, 0.7, 0.9, 1.0),
    shading='smooth',
    parent=view.scene
)
spacecraft_transform = MatrixTransform()
spacecraft.transform = spacecraft_transform

# thrust plume
plume_vertices, plume_faces = create_cone_geometry(length=0.6, base_radius=0.16)

thrust_plume = scene.visuals.Mesh(
    vertices=plume_vertices,
    faces=plume_faces,
    color=(1.0, 0.5, 0.0, 0.0), 
    shading=None,
    parent=view.scene
)
thrust_plume.set_gl_state('translucent', blend=True, depth_test=True)
thrust_plume.visible = False
thrust_plume_transform = MatrixTransform()
thrust_plume.transform = thrust_plume_transform

world_axis_length = 0.8
world_axis_offset = np.array([0, 0, 0])
world_axis_dirs = [
    np.array([world_axis_offset, world_axis_offset + [world_axis_length, 0, 0]]),
    np.array([world_axis_offset, world_axis_offset + [0, world_axis_length, 0]]),
    np.array([world_axis_offset, world_axis_offset + [0, 0, world_axis_length]]),
]

world_axis_lines = [
    Line(world_axis_dirs[0], color=hex_to_rgb('#f25060'), width=2, parent=view.scene),
    Line(world_axis_dirs[1], color=hex_to_rgb('#50f28b'), width=2, parent=view.scene),
    Line(world_axis_dirs[2], color=hex_to_rgb('#50c4f2'), width=2, parent=view.scene),
]
world_axes_transform = MatrixTransform()
for world_axis in world_axis_lines:
    world_axis.transform = world_axes_transform

from vispy.scene.visuals import Rectangle

world_axis_box = Rectangle(center=(0, 0), width=120, height=130, 
                           color=(0, 0, 0, 0.7), border_color=None,
                           parent=canvas.scene)
body_axis_box = Rectangle(center=(0, 0), width=120, height=130,
                          color=(0, 0, 0, 0.7), border_color=None,
                          parent=canvas.scene)

world_x_text = Text('world x', parent=canvas.scene, color=hex_to_rgb('#f25060'), bold=True, font_size=10, face='Tahoma')
world_y_text = Text('world y', parent=canvas.scene, color=hex_to_rgb('#50f28b'), bold=True, font_size=10, face='Tahoma')
world_z_text = Text('world z', parent=canvas.scene, color=hex_to_rgb('#50c4f2'), bold=True, font_size=10, face='Tahoma')

alt_text = Text('', parent=canvas.scene, color='yellow', bold=True, font_size=12, face='Tahoma')
alt_text.anchors = ('left', 'top')

time_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=12, face='Tahoma')
time_text.anchors = ('left', 'top')

throttle_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=9, face='Tahoma')
throttle_text.anchors = ('left', 'top')

prop_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=9, face='Tahoma')
prop_text.anchors = ('left', 'top')


pos_text = Text('', parent=canvas.scene, color='yellow', bold=True, font_size=7, face='Tahoma')
pos_text.anchors = ('right', 'top')

vel_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=7, face='Tahoma')
vel_text.anchors = ('right', 'top')

quat_text = Text('', parent=canvas.scene, color='white', bold=True, font_size=7, face='Tahoma')
quat_text.anchors = ('right', 'top')

pitch_text = Text('', parent=canvas.scene, color=hex_to_rgb("#48fe57"), bold=True, font_size=7, face='Tahoma')
pitch_text.anchors = ('right', 'top')
yaw_text = Text('', parent=canvas.scene, color=hex_to_rgb("#37c3fa"), bold=True, font_size=7, face='Tahoma')
yaw_text.anchors = ('right', 'top')
roll_text = Text('', parent=canvas.scene, color=hex_to_rgb("#ff5160"), bold=True, font_size=7, face='Tahoma')
roll_text.anchors = ('right', 'top')


x_axis_text = Text('body x', parent=canvas.scene, color='red', bold=True, font_size=10, face='Tahoma')
y_axis_text = Text('body y', parent=canvas.scene, color='green', bold=True, font_size=10, face='Tahoma')
z_axis_text = Text('body z', parent=canvas.scene, color='blue', bold=True, font_size=10, face='Tahoma')

def update_text_positions():
    w, h = canvas.size
    
    alt_text.pos = 10, 50
    time_text.pos = 10, 90
    prop_text.pos = 10, 125
    throttle_text.pos = 10, 160

    pos_text.pos = w - 20, 35
    vel_text.pos = w - 20, 60
    quat_text.pos = w - 20, 85
    pitch_text.pos = w - 20, 110
    yaw_text.pos = w - 20, 135
    roll_text.pos = w - 20, 160
    
    x_axis_text.pos = w - 60, h - 100
    y_axis_text.pos = w - 60, h - 60
    z_axis_text.pos = w - 60, h - 20

    world_x_text.pos = 60, h - 100
    world_y_text.pos = 60, h - 60
    world_z_text.pos = 60, h - 20
    
    world_axis_box.center = (60, h - 60)
    body_axis_box.center = (w - 60, h - 60)

@canvas.events.resize.connect
def on_resize(event):
    update_text_positions()

update_text_positions()

view.camera = scene.TurntableCamera(fov=60, distance=2, elevation=10, azimuth=45)

POSITION_SCALE = 9 # tuned
GRID_VISIBILITY_THRESHOLD = 2000.0
GROUND_VISIBILITY_THRESHOLD = 5000.0
STARFIELD_FADE_START = 5000.0
STARFIELD_FADE_END = 6000.0

current_frame = [0]
paused = [False]
playback_speed = [1]  # 1x, 2x, 5x

@canvas.events.key_press.connect
def on_key_press(event):
    if event.key == 'Space':
        # toggle pause
        paused[0] = not paused[0]
        print(f"{'Paused' if paused[0] else 'Playing'}")
    elif event.key == 'Right':
        # skip forward 10 frames
        current_frame[0] = (current_frame[0] + 10) % len(times)
        print(f"Skipped forward to frame {current_frame[0]}")
    elif event.key == 'Left':
        # skip backward 10 frames
        current_frame[0] = (current_frame[0] - 10) % len(times)
        print(f"Skipped backward to frame {current_frame[0]}")
    elif event.key == 'Up':
        # increase playback speed
        playback_speed[0] = min(playback_speed[0] * 2, 32)
        print(f"Playback speed: {playback_speed[0]}x")
    elif event.key == 'Down':
        # decrease playback speed
        playback_speed[0] = max(playback_speed[0] / 2, 0.25)
        print(f"Playback speed: {playback_speed[0]}x")
    elif event.key == 'R':
        # reset to beginning
        current_frame[0] = 0
        print("Reset to beginning")

def update(ev):
    if paused[0]:
        return
    
    i = current_frame[0] % len(times)
    current_frame[0] = (current_frame[0] + int(playback_speed[0])) % len(times)
    
    q = quat[i]
    eul = euler_ang[i]
    p = pos[i]
    v = vel[i]
    thr = throttle[i]
    prp = prop[i]

    rot = R.from_quat([q[1], q[2], q[3], -q[0]]).as_matrix()
    spacecraft_pos = p / POSITION_SCALE

    grid.visible = p[2] <= GRID_VISIBILITY_THRESHOLD

    if p[2] > STARFIELD_FADE_START:
        starfield.visible = True
        if p[2] < STARFIELD_FADE_END:
            # fade in between 5000-6000m
            alpha = (p[2] - STARFIELD_FADE_START) / (STARFIELD_FADE_END - STARFIELD_FADE_START)
            star_colors[:, 3] = star_base_alpha * alpha
            starfield.set_data(pos=star_positions, face_color=star_colors)
        else:
            # visible above 6000m
            star_colors[:, 3] = star_base_alpha
            starfield.set_data(pos=star_positions, face_color=star_colors)
    else:
        starfield.visible = False

    low_color = np.array(hex_to_rgb("#255529"))  
    high_color = np.array(hex_to_rgb("#000000")) 

    if p[2] > GROUND_VISIBILITY_THRESHOLD:
        ground.visible = False
    elif p[2] > (GROUND_VISIBILITY_THRESHOLD - 1000):
        ground.visible = True
        t = (p[2] - (GROUND_VISIBILITY_THRESHOLD - 1000)) / 1000
        new_color = (1 - t) * low_color + t * high_color
        ground._mesh.color = new_color
        ground._mesh.update()
    else:
        ground.visible = True
        ground._mesh.color = low_color
        ground._mesh.update()

    combined_rot = rot @ axes_calibrate_rot_3x3
    spacecraft_transform.reset()
    rot_matrix_4x4 = np.eye(4)
    rot_matrix_4x4[:3, :3] = combined_rot
    spacecraft_transform.matrix = rot_matrix_4x4
    spacecraft_transform.translate(spacecraft_pos)

    # Update thrust plume
    if thr > 0.01:  # Show plume if throttle is above threshold
        thrust_plume.visible = True
        
        # Set transparency based on throttle level
        alpha = 0.05 + (thr * 0.9)
        
        # Update color with new alpha
        thrust_plume.color = (1.0, 0.5, 0.0, alpha)
        
        # Apply same transformation as spacecraft
        thrust_plume_transform.reset()
        thrust_plume_transform.matrix = rot_matrix_4x4
        thrust_plume_transform.translate(spacecraft_pos)
    else:
        thrust_plume.visible = False

    axes_transform.reset()
    axes_transform.matrix = rot_matrix_4x4
    axes_transform.translate(spacecraft_pos)

    world_axes_transform.reset()
    world_axes_transform.translate(spacecraft_pos)

    starfield_transform.reset()
    starfield_transform.translate(spacecraft_pos)

    view.camera.center = tuple(spacecraft_pos)

    alt_text.text = f"Alt: {p[2]:.2f} m"
    time_text.text = f"Time: {times[i]:.2f} s"
    pos_text.text = f"pos(x y z): [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}] m"
    vel_text.text = f"vel(x y z): [{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}] m/s"
    prop_text.text = f"Fuel tank: {prp:.2f} %"
    throttle_text.text = f"Thrust: {(thr*100):.2f} %"
    quat_text.text = f"quat(w x y z): [{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]"
    pitch_text.text = f"pitch: {eul[0]:.2f}°"   
    yaw_text.text = f"yaw: {eul[1]:.2f}°"       
    roll_text.text = f"roll: {eul[2]:.2f}°"


    canvas.update()

timer = app.Timer(0.01, connect=update, start=True)
app.run()