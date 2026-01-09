# SimVisGraph
# Used for plotting sim data off of csv

import pandas as pd
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import yaml

# csv config settings
simData = pd.read_csv("simulation_data.csv")
columns = list(simData.columns)

# ensure that the preset file has been formatted properly to avoid crashes
def validate_presets(presets: dict, columns: list[str]):
    valid_presets = {}

    for preset_name, preset in presets.items():
        if "plots" not in preset:
            raise ValueError(f"Preset '{preset_name}' missing 'plots'")

        for i, plot in enumerate(preset["plots"]):
            if "type" not in plot:
                raise ValueError(f"{preset_name}[{i}] missing 'type'")

            plot_type = plot["type"]
            if plot_type not in ("2d", "3d"):
                raise ValueError(
                    f"{preset_name}[{i}] invalid type '{plot_type}'"
                )

            for axis in ("x", "y"):
                if axis not in plot:
                    raise ValueError(
                        f"{preset_name}[{i}] missing '{axis}'"
                    )
                if plot[axis] not in columns:
                    raise ValueError(
                        f"{preset_name}[{i}] column '{plot[axis]}' not in CSV"
                    )

            if plot_type == "3d":
                if "z" not in plot:
                    raise ValueError(
                        f"{preset_name}[{i}] 3d plot missing 'z'"
                    )
                if plot["z"] not in columns:
                    raise ValueError(
                        f"{preset_name}[{i}] column '{plot['z']}' not in CSV"
                    )

        valid_presets[preset_name] = preset

    return valid_presets

# load preset file
with open("presets.yaml", "r") as f:
    raw_presets = yaml.safe_load(f)["presets"]

presets = validate_presets(raw_presets, columns)
preset_names = list(presets.keys())

# general plotting functions for preset and manual plotting
def plot_2d(x_col, y_col, title=None):
    fig, ax = plt.subplots()
    ax.plot(simData[x_col], simData[y_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_title(title or f"{y_col} vs {x_col}")
    ax.grid(True)
    plt.show(block=False)
def plot_3d(x_col, y_col, z_col, title=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(simData[x_col], simData[y_col], simData[z_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_zlabel(z_col)
    ax.set_title(title or "3D Plot")
    plt.show(block=False)

# graph based on preset file
def run_preset(preset_name):
    for plot in presets[preset_name]["plots"]:
        if plot["type"] == "2d":
            plot_2d(plot["x"], plot["y"], plot.get("title"))
        else:
            plot_3d(plot["x"], plot["y"], plot["z"], plot.get("title")
            )

# manual plotting functions
def make_2d_plot():
    plot_2d(x_var.get(), y_var.get())
def make_3d_plot():
    plot_3d(x_var.get(), y_var.get(), z_var.get())
def update_3d_button(*args):
    plot3d_button.config(state=tk.NORMAL if z_var.get() != "None" else tk.DISABLED)

# window config settings
root = tk.Tk()
root.title("SimVisGraph")
root.geometry("250x380")

# x-axis dropdown menu
tk.Label(root, text="X Axis").pack(pady=(10, 0))
x_var = tk.StringVar(value=columns[0])
ttk.Combobox(root, textvariable=x_var, values=columns, state="readonly").pack()

# y-axis dropdown menu
tk.Label(root, text="Y Axis").pack(pady=(10, 0))
y_var = tk.StringVar(value=columns[1] if len(columns) > 1 else columns[0])
ttk.Combobox(root, textvariable=y_var, values=columns, state="readonly").pack()

# z-axis dropdown menu
tk.Label(root, text="Z Axis (optional)").pack(pady=(10, 0))
z_var = tk.StringVar(value="None")
ttk.Combobox(root, textvariable=z_var, values=["None"] + columns, state="readonly").pack()

# manual plot buttons
tk.Button(root, text="Make 2D Plot", command=make_2d_plot).pack(pady=10)
plot3d_button = tk.Button(root, text="Make 3D Plot", command=make_3d_plot, state=tk.DISABLED)
plot3d_button.pack()
z_var.trace_add("write", update_3d_button)

# preset plotting buttons
ttk.Separator(root, orient="horizontal").pack(fill="x", pady=15)

tk.Label(root, text="Presets").pack()
preset_var = tk.StringVar(value=preset_names[0])
ttk.Combobox(root, textvariable=preset_var, values=preset_names, state="readonly").pack()

tk.Button(root, text="Load Preset", command=lambda: run_preset(preset_var.get())).pack(pady=10)

root.mainloop()