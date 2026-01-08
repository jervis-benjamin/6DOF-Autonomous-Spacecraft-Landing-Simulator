# SimVisGraph
# Used for plotting sim data off of csv

import pandas as pd
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt

# csv config settings
simData = pd.read_csv("simulation_data.csv")
columns = list(simData.columns)

# csv config settings
root = tk.Tk()
root.title("CSV Plotter")
root.geometry("300x200")

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


def make_plot():
    # creates a matplotlib plot of the chosen x and y columns

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

# make plot button
tk.Button(root, text="Make Plot", command=make_plot).pack(pady=20)

root.mainloop()
