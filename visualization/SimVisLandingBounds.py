import os
import glob
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class LandingHeatmapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Landing Bounds")
        self.root.geometry("300x250") 

        self.df_results = None
        self.mc_columns = []
        self.descent_rate_threshold = 1.5  # descent velocity magnitude threshold in m/s

        self._setup_ui()

    def _setup_ui(self):
        # buttons and tools to load data and push plots

        frame = ttk.Frame(self.root, padding="15")
        frame.pack(fill=tk.BOTH, expand=True)

        # load data folder
        self.btn_load = ttk.Button(frame, text="Load Data Folder", command=self.load_data)
        self.btn_load.pack(fill=tk.X, pady=(0, 10))

        lbl_descent = ttk.Label(frame, text="Descent Rate Threshold (m/s):")
        lbl_descent.pack(anchor=tk.W)
        
        self.var_descent_rate = tk.DoubleVar(value=1.5)
        self.entry_descent = ttk.Entry(frame, textvariable=self.var_descent_rate)
        self.entry_descent.pack(fill=tk.X, pady=(0, 10))

        lbl_dd = ttk.Label(frame, text="Heatplot Variable:")
        lbl_dd.pack(anchor=tk.W)

        self.var_dropdown = tk.StringVar()
        self.combo_mc = ttk.Combobox(frame, textvariable=self.var_dropdown, state="readonly")
        self.combo_mc['values'] = ('None',)
        self.combo_mc.current(0)
        self.combo_mc.pack(fill=tk.X, pady=(0, 15))

        self.btn_plot = ttk.Button(frame, text="Create Landing Plot", command=self.create_plot, state=tk.DISABLED)
        self.btn_plot.pack(fill=tk.X, pady=(0, 5))

        self.lbl_status = ttk.Label(frame, text="No data loaded", font=("Arial", 8), foreground="gray")
        self.lbl_status.pack(side=tk.BOTTOM)

    def load_data(self):
        folder_path = filedialog.askdirectory()
        if not folder_path:
            return

        self.lbl_status.config(text="Processing files...", foreground="blue")
        self.root.update()

        parquet_files = glob.glob(os.path.join(folder_path, "*.parquet"))
        
        if not parquet_files:
            messagebox.showerror("Error", "No .parquet files found.")
            self.lbl_status.config(text="No files found", foreground="red")
            return

        data_list = []

        time_col = "time (s)"
        
        for file in parquet_files:
            df = pd.read_parquet(file)
            
            required = ["posX (m)", "posY (m)", "posZ (m)", "velX (m/s)", "velY (m/s)", "velZ (m/s)"]
            if not all(col in df.columns for col in required):
                continue

            # check if we actually landed by seeing if the z pos is too high
            final_row = df.iloc[-1]
            if final_row["posZ (m)"] < 5:
                # calculate velocity magnitude at impact (5 seconds before the sim ends)
                t_final = df[time_col].iloc[-1]
                t_target = t_final - 5.01
                impact_idx = (df[time_col] - t_target).abs().idxmin()
                impact_row = df.loc[impact_idx]

                vel_mag = np.sqrt(
                    impact_row["velX (m/s)"]**2 + 
                    impact_row["velY (m/s)"]**2 + 
                    impact_row["velZ (m/s)"]**2
                )
                
                entry = {
                    "posX": final_row["posX (m)"],
                    "posY": final_row["posY (m)"],
                    "vel_magnitude": vel_mag
                }

                # since all of the rows are the same for mc variables, we will only fill in using the first row 
                first_row = df.iloc[0]
                mc_cols = [c for c in df.columns if c.startswith("mc_")]
                for mc_col in mc_cols:
                    entry[mc_col] = first_row[mc_col]

                data_list.append(entry)

        if not data_list:
            messagebox.showwarning("Warning", "No valid landings (posZ < 5) found.")
            self.lbl_status.config(text="No valid data", foreground="red")
            return

        self.df_results = pd.DataFrame(data_list)
        
        # dropdown menu options
        self.mc_columns = [c for c in self.df_results.columns if c.startswith("mc_")]
        self.combo_mc['values'] = ['None'] + sorted(self.mc_columns)
        self.combo_mc.current(0)
        
        self.btn_plot.config(state=tk.NORMAL)
        self.lbl_status.config(text=f"Loaded {len(self.df_results)} runs", foreground="green")

    def create_plot(self):
        if self.df_results is None or self.df_results.empty:
            return
        
        fig, ax = plt.subplots(figsize=(8, 6))
        
        x = self.df_results["posX"]
        y = self.df_results["posY"]
        selected_mc = self.var_dropdown.get()
        
        # get descent rate threshold
        try:
            threshold = self.var_descent_rate.get()
        except:
            threshold = 1.5

        # parse data by descent rate
        high_vel_mask = self.df_results["vel_magnitude"] > threshold
        normal_vel_mask = ~high_vel_mask

        if selected_mc == "None" or selected_mc not in self.df_results.columns:
            # cirlces for nominal runs
            ax.scatter(x[normal_vel_mask], y[normal_vel_mask], alpha=0.6, edgecolors='w', s=50, color='royalblue', label='Normal Landings', marker='o')
            # mark runs outside threshold velocity with a triangle
            ax.scatter(x[high_vel_mask], y[high_vel_mask], alpha=0.6, edgecolors='w',s=50, color='orange', label=f'High Velocity (>{threshold} m/s)', marker='^')
            ax.set_title("Landing Dispersion")
        else:
            c = self.df_results[selected_mc]
            # cirlces for nominal runs
            if normal_vel_mask.any():
                sc1 = ax.scatter(x[normal_vel_mask], y[normal_vel_mask], c=c[normal_vel_mask], cmap='viridis', alpha=0.7, edgecolors='k', linewidth=0.3, s=50, marker='o', label='Normal Landings')
            # mark runs outside threshold velocity with a triangle
            if high_vel_mask.any():
                sc2 = ax.scatter(x[high_vel_mask], y[high_vel_mask], c=c[high_vel_mask], cmap='viridis', alpha=0.7, edgecolors='k', linewidth=0.3, s=50, marker='^', label=f'High Velocity (>{threshold} m/s)')
            
            ax.set_title(f"Landing Dispersion by {selected_mc}")
            # Use the first scatter for colorbar
            if normal_vel_mask.any():
                cbar = plt.colorbar(sc1, ax=ax)
            elif high_vel_mask.any():
                cbar = plt.colorbar(sc2, ax=ax)
            if normal_vel_mask.any() or high_vel_mask.any():
                cbar.set_label(selected_mc)

        # draw ellipse
        self._draw_confidence_ellipse(x, y, ax, n_std=3.0, edgecolor='red')

        ax.set_xlabel("posX (m)")
        ax.set_ylabel("posY (m)")
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.axis('equal') 
        ax.legend()
        plt.show()

    def _draw_confidence_ellipse(self, x, y, ax, n_std=3.0, **kwargs):
        # calculates ellipse

        if len(x) < 2: return

        # covariance matrix for landing elipse
        cov = np.cov(x, y)
        lambda_, v = np.linalg.eig(cov)
        lambda_ = np.sqrt(lambda_)
        width = lambda_[0] * n_std * 2
        height = lambda_[1] * n_std * 2
     
        if lambda_[0] > lambda_[1]:
            angle = np.degrees(np.arctan2(v[1, 0], v[0, 0]))
            width = 2 * n_std * lambda_[0]
            height = 2 * n_std * lambda_[1]
        else:
            angle = np.degrees(np.arctan2(v[1, 1], v[0, 1]))
            width = 2 * n_std * lambda_[1]
            height = 2 * n_std * lambda_[0]

        mean_x, mean_y = np.mean(x), np.mean(y)

        # generate ellipse
        ellipse = Ellipse((mean_x, mean_y), width=width, height=height, angle=angle, facecolor='none', linestyle='--', linewidth=2, **kwargs)
        
        ax.add_patch(ellipse)
        
        # legend
        ax.plot([], [], color=kwargs.get('edgecolor', 'red'), linestyle='--', label=f'{int(n_std)}-Sigma')

if __name__ == "__main__":
    root = tk.Tk()
    app = LandingHeatmapApp(root)
    root.mainloop()