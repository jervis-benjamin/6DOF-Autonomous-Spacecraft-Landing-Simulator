# SimVisGraph.py
# Used for plotting sim data off of csv

import pandas as pd
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import yaml
import os
from tkinter import filedialog, messagebox

# window config settings
root = tk.Tk()
root.title("SimVisGraph")
root.geometry("250x760")

simData = None
columns = []
presets = {}

def load_data():
    global simData, columns, presets
    
    file_path = filedialog.askopenfilename(
        title="Select Simulation Data",
        filetypes=[("Parquet Files", "*.parquet"), ("All Files", "*.*")]
    )
    
    if not file_path:
        return

    try:
        # load data
        simData = pd.read_parquet(file_path)
        columns = list(simData.columns)
        
        update_dropdowns()
        
        filename = os.path.basename(file_path)
        status_label.config(text=f"Loaded: {filename}", fg="green")
        
        btn_2d.config(state=tk.NORMAL)
        
        load_presets()
        
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load file:\n{str(e)}")
        status_label.config(text="Error loading file", fg="red")

def update_dropdowns():
    combos = [combo_x, combo_y, combo_multi_x, combo_multi_y]
    for c in combos:
        c['values'] = columns
        if columns:
            c.current(0)
            
    combo_z['values'] = ["None"] + columns
    combo_z.current(0)
  
    added_y_listbox.delete(0, tk.END)
    update_multiplot_button()

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
def load_presets():
    global presets
    if not os.path.exists("presets.yaml"):
        return

    try:
        with open("presets.yaml", "r") as f:
            raw = yaml.safe_load(f)
            if raw and "presets" in raw:
                presets = validate_presets(raw["presets"], columns)
                
                preset_names = list(presets.keys())
                if preset_names:
                    combo_preset['values'] = preset_names
                    combo_preset.current(0)
                    btn_preset.config(state=tk.NORMAL)
                else:
                    combo_preset['values'] = ["No valid presets"]
                    btn_preset.config(state=tk.DISABLED)
    except Exception as e:
        print(f"Preset load error: {e}")

# general plotting functions for preset and manual plotting
def plot_2d(x_col, y_col, title=None):
    # plt.close("all") # to prevent crashes
    fig, ax = plt.subplots()
    ax.plot(simData[x_col], simData[y_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_title(title or f"{y_col} vs {x_col}")
    ax.grid(True)
    plt.show(block=False)

def plot_3d(x_col, y_col, z_col, title=None):
    # plt.close("all") # to prevent crashes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(simData[x_col], simData[y_col], simData[z_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_zlabel(z_col)
    ax.set_title(title or "3D Plot")
    plt.show(block=False)

# for plotting multiple axis in one plot
def plot_2d_multiplot(x_col, y_cols):
    fig, ax = plt.subplots()
    axes = [ax]

    base_color_cycle = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for i, y in enumerate(y_cols):
        if i == 0:
            curr_ax = ax
            curr_ax.plot(simData[x_col], simData[y],
                         label=y,
                         color=base_color_cycle[i % 10])
            curr_ax.set_ylabel(y,
                               color=base_color_cycle[i % 10])
            curr_ax.tick_params(axis="y",
                                colors=base_color_cycle[i % 10])
        else:
            curr_ax = ax.twinx()
            offset = 0.1 * (i - 1)  

            curr_ax.spines["right"].set_position(("axes", 1 + offset))
            curr_ax.set_frame_on(True)
            curr_ax.patch.set_visible(False)

            color = base_color_cycle[i % 10]
            curr_ax.plot(simData[x_col], simData[y], label=y, color=color)
            curr_ax.set_ylabel(y, color=color)
            curr_ax.tick_params(axis="y", colors=color)
            curr_ax.spines["right"].set_edgecolor(color)

            axes.append(curr_ax)

    ax.set_xlabel(x_col)
    ax.set_title("2D Multiplot")

    lines, labels = [], []
    for a in axes:
        l, lab = a.get_legend_handles_labels()
        lines.extend(l)
        labels.extend(lab)

    ax.legend(lines, labels, loc="best")
    ax.grid(True)
    fig.tight_layout()
    plt.show(block=False)
  

# graph based on preset file
def run_preset(preset_name):
    for plot in presets[preset_name]["plots"]:
        if plot["type"] == "2d":
            plot_2d(plot["x"], plot["y"], plot.get("title"))
        else:
            plot_3d(plot["x"], plot["y"], plot["z"], plot.get("title"))


# file selection
file_frame = tk.Frame(root, pady=10)
file_frame.pack(fill=tk.X)
tk.Button(file_frame, text="Load Parquet File", command=load_data, bg="#e1e1e1").pack()
status_label = tk.Label(file_frame, text="No file loaded", font=("Arial", 8), fg="gray")
status_label.pack()

ttk.Separator(root, orient="horizontal").pack(fill="x", pady=5)

tk.Label(root, text="X Axis").pack(pady=(5, 0))
x_var = tk.StringVar()
combo_x = ttk.Combobox(root, textvariable=x_var, state="readonly")
combo_x.pack()

tk.Label(root, text="Y Axis").pack(pady=(5, 0))
y_var = tk.StringVar()
combo_y = ttk.Combobox(root, textvariable=y_var, state="readonly")
combo_y.pack()

tk.Label(root, text="Z Axis (optional)").pack(pady=(5, 0))
z_var = tk.StringVar(value="None")
combo_z = ttk.Combobox(root, textvariable=z_var, state="readonly")
combo_z.pack()

def update_3d_button(*args):
    if simData is not None and z_var.get() != "None":
        btn_3d.config(state=tk.NORMAL)
    else:
        btn_3d.config(state=tk.DISABLED)

z_var.trace_add("write", update_3d_button)

def make_2d_plot():
    if simData is not None:
        plot_2d(x_var.get(), y_var.get())

def make_3d_plot():
    if simData is not None:
        plot_3d(x_var.get(), y_var.get(), z_var.get())

btn_2d = tk.Button(root, text="Make 2D Plot", command=make_2d_plot, state=tk.DISABLED)
btn_2d.pack(pady=10)

btn_3d = tk.Button(root, text="Make 3D Plot", command=make_3d_plot, state=tk.DISABLED)
btn_3d.pack()

# presets
ttk.Separator(root, orient="horizontal").pack(fill="x", pady=15)
tk.Label(root, text="Presets").pack()
preset_var = tk.StringVar()
combo_preset = ttk.Combobox(root, textvariable=preset_var, state="readonly")
combo_preset.pack()

btn_preset = tk.Button(root, text="Load Preset", command=lambda: run_preset(preset_var.get()), state=tk.DISABLED)
btn_preset.pack(pady=10)

# multiplot
ttk.Separator(root, orient="horizontal").pack(fill="x", pady=15)
tk.Label(root, text="2D Multiplot").pack()

tk.Label(root, text="X Axis").pack()
multi_x_var = tk.StringVar()
combo_multi_x = ttk.Combobox(root, textvariable=multi_x_var, state="readonly")
combo_multi_x.pack()

tk.Label(root, text="Available Y Axis").pack()
multi_y_var = tk.StringVar()
combo_multi_y = ttk.Combobox(root, textvariable=multi_y_var, state="readonly")
combo_multi_y.pack()

tk.Label(root, text="Multiplot Y Axes").pack()
added_y_listbox = tk.Listbox(root, height=5)
added_y_listbox.pack()

def update_multiplot_button(*_):
    count = added_y_listbox.size()
    if simData is not None and count >= 2:
        btn_multi.config(state=tk.NORMAL)
    else:
        btn_multi.config(state=tk.DISABLED)

def add_multiplot_y():
    if simData is None: return
    y = multi_y_var.get()
    existing = added_y_listbox.get(0, tk.END)
    if y and y not in existing:
        added_y_listbox.insert(tk.END, y)
    update_multiplot_button()

def remove_multiplot_y():
    for idx in reversed(added_y_listbox.curselection()):
        added_y_listbox.delete(idx)
    update_multiplot_button()

def make_multiplot():
    y_cols = list(added_y_listbox.get(0, tk.END))
    plot_2d_multiplot(multi_x_var.get(), y_cols)

tk.Button(root, text="Add Y Axis", command=add_multiplot_y).pack(pady=2)
tk.Button(root, text="Remove Selected Y", command=remove_multiplot_y).pack(pady=2)

btn_multi = tk.Button(root, text="Make Multiplot", state=tk.DISABLED, command=make_multiplot)
btn_multi.pack(pady=5)

root.mainloop()