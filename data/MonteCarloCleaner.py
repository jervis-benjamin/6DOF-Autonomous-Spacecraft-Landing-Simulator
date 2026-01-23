# cleans monte_carlo folder of .bin and .parquet
# this file is likely causing the terminal printout issue

import os
import glob
import sys

def clean_directory():

    target_dir = r"C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/monte_carlo"
    

    if len(target_dir) < 20: 
        print("Path is too short, potentially dangerous.")
        return

    
    if "monte_carlo" not in target_dir:
        print("Target directory does not contain 'monte_carlo', check again")
        return

    if not os.path.exists(target_dir):
        print(f"Directory not found:")
        return

    files = glob.glob(os.path.join(target_dir, "*"))
    
    if not files:
        print("Folder is already empty.")
        return

    print(f"Cleaning {len(files)} files from: {target_dir}")
    
    deleted_count = 0
    for f in files:

        if f.endswith(".bin") or f.endswith(".parquet"):
            os.remove(f)
            deleted_count += 1
    
    print(f"Successfully emptied monte_carlo folder")

if __name__ == "__main__":
    clean_directory()