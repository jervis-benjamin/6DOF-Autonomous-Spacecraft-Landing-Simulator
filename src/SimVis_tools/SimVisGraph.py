# SimVisGraph
# Used for plotting sim data off of csv

import pandas as pd
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 

# csv config settings
simData = pd.read_csv("simulation_data.csv")
columns = list(simData.columns)

# csv config settings
root = tk.Tk()
root.title("CSV Plotter")
root.geometry("260x280")

# x-axis dropdown menu
tk.Label(root, text="X Axis").pack(pady=(10, 0))
x_var = tk.StringVar(value=columns[0])
x_dropdown = ttk.Combobox(root, textvariable=x_var, values=columns, state="readonly")
x_dropdown.pack()

# y-axis dropdown menu
tk.Label(root, text="Y Axis").pack(pady=(10, 0))
y_var = tk.StringVar(value=columns[1] if len(columns) > 1 else columns[0])
y_dropdown = ttk.Combobox(root, textvariable=y_var, values=columns, state="readonly")
y_dropdown.pack()

# z-axis dropdown menu
tk.Label(root, text="Z Axis (optional)").pack(pady=(10, 0))
z_columns = ["None"] + columns # setting the default to none since it is optional
z_var = tk.StringVar(value="None")
z_dropdown = ttk.Combobox(root, textvariable=z_var, values=z_columns, state="readonly")
z_dropdown.pack()


def make_2d_plot():
    # creates a 2d plot of the chosen x and y columns

    x_col = x_var.get()
    y_col = y_var.get()

    fig, ax = plt.subplots()
    ax.plot(simData[x_col], simData[y_col])
    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_title(f"{y_col} vs {x_col}")
    ax.grid(True)

    # keep plot maker window visible in the background
    plt.show(block=False)

def make_3d_plot():
    # creates a 3d plot of the chosen x, y, and z columns

    x_col = x_var.get()
    y_col = y_var.get()
    z_col = z_var.get()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        simData[x_col],
        simData[y_col],
        simData[z_col]
    )

    ax.set_xlabel(x_col)
    ax.set_ylabel(y_col)
    ax.set_zlabel(z_col)
    ax.set_title(f"3D Plot")

    # keep plot maker window visible in the background
    plt.show(block=False)

# disable button if a z column was not chosen
def update_3d_button(*args):
    if z_var.get() == "None":
        plot3d_button.config(state=tk.DISABLED)
    else:
        plot3d_button.config(state=tk.NORMAL)

# plot buttons
tk.Button(root, text="Make 2D Plot", command=make_2d_plot).pack(pady=20)
plot3d_button = tk.Button(root, text="Make 3D Plot", command=make_3d_plot, state=tk.DISABLED)
plot3d_button.pack()
z_var.trace_add("write", update_3d_button)

root.mainloop()