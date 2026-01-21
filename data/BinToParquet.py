# ~ BinToParquet.py ~
# Owns: 
# - Simulation_data.bin formatting
# *Ultized Claude for general code functions as converting file types is not a main objective of this project

import pandas as pd
import numpy as np
import re
import os
import glob

def get_run_config_mode(header_path):
    """
    Reads RunConfig.h to determine if monteCarloRun is true or false.
    """
    if not os.path.exists(header_path):
        print(f"WARNING: Could not find {header_path}. Defaulting to Single Run mode.")
        return False

    with open(header_path, 'r') as f:
        content = f.read()
    
    # Regex to find: bool monteCarloRun = true; (or false)
    match = re.search(r'bool\s+monteCarloRun\s*=\s*(true|false)', content, re.IGNORECASE)
    
    if match:
        val = match.group(1).lower()
        is_mc = (val == 'true')
        print(f"RunConfig detected: monteCarloRun = {is_mc}")
        return is_mc
    
    print("WARNING: Could not find 'bool monteCarloRun' in RunConfig.h. Defaulting to False.")
    return False

def parse_cpp_struct(header_path):
    dtype_list = []
    column_map = {} 

    with open(header_path, 'r') as f:
        content = f.read()

    match = re.search(r'struct\s+RecordData\s*\{(.*?)\};', content, re.DOTALL)
    if not match:
        raise ValueError("Could not find 'struct RecordData'")
    
    body = match.group(1)

    regex = r'(\w+)\s+(\w+)(?:\[(\d+)\])?;\s*(?://\s*\[cols:\s*(.*?)\])?'

    for line in body.split('\n'):
        line = line.strip()
        if not line or line.startswith('#'): continue

        m = re.search(regex, line)
        if m:
            ctype, name, arr_size, custom_cols = m.groups()
            
            np_type = 'f8'
            if ctype == 'int': np_type = 'i4'
            elif ctype == 'bool': np_type = '?'

            if arr_size:
                size = int(arr_size)
                dtype_list.append((name, np_type, (size,)))
                
                if custom_cols:
                    cols = [c.strip() for c in custom_cols.split(',')]
                    if len(cols) != size:
                        print(f"WARNING: {name} has {size} elements but {len(cols)} labels. Using defaults.")
                        column_map[name] = [f"{name}_{i}" for i in range(size)]
                    else:
                        column_map[name] = cols
                else:
                    column_map[name] = [f"{name}_{i}" for i in range(size)]
            else:
                dtype_list.append((name, np_type))
                
                if custom_cols:
                    column_map[name] = [custom_cols.strip()]
                else:
                    column_map[name] = [name]

    return np.dtype(dtype_list), column_map

def convert_bin_to_parquet(bin_file, parquet_file, header_file, quiet=False):
    if not quiet:
        print(f"Processing {os.path.basename(bin_file)}...")
    
    if not os.path.exists(bin_file):
        if not quiet: print(f"File not found: {bin_file}")
        return

    try:
        dt, col_map = parse_cpp_struct(header_file)
        data = np.fromfile(bin_file, dtype=dt)
        
        if data.size == 0:
            print(f"Skipping empty file: {bin_file}")
            return

        df_data = {}
        for name in dt.names:
            raw_col = data[name]
            target_names = col_map[name]
            
            if raw_col.ndim > 1: 
                for i, col_name in enumerate(target_names):
                    df_data[col_name] = raw_col[:, i]
            else: 
                df_data[target_names[0]] = raw_col
                
        df = pd.DataFrame(df_data)
        df.to_parquet(parquet_file)
        
        if os.path.exists(parquet_file) and os.path.getsize(parquet_file) > 0:
            os.remove(bin_file)
            if not quiet:
                print(f" -> Converted & Deleted .bin")
        else:
            print(f"WARNING: Parquet verification failed for {bin_file}")
            
    except Exception as e:
        print(f"ERROR processing {bin_file}: {e}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # path assumptions:
    # data/BinToParquet.py
    # data/RecordData.h
    # include/RunConfig.h
    
    record_header = os.path.join(script_dir, "RecordData.h")
    
    run_config_header = os.path.abspath(os.path.join(script_dir, "..", "include", "RunConfig.h"))

  
    is_monte_carlo = get_run_config_mode(run_config_header)

    
    if is_monte_carlo:
        print("\n--- MODE: Monte Carlo Batch Processing ---")
        mc_dir = os.path.join(script_dir, "monte_carlo")
        
        if os.path.exists(mc_dir):
            mc_files = glob.glob(os.path.join(mc_dir, "*.bin"))
            if mc_files:
                print(f"Found {len(mc_files)} files. Converting...")
                for bin_f in mc_files:
                    parquet_f = os.path.splitext(bin_f)[0] + ".parquet"
                    convert_bin_to_parquet(bin_f, parquet_f, record_header, quiet=True)
                print("Batch processing complete.")
            else:
                print("No .bin files found in monte_carlo folder (Sim might have failed or folder is empty).")
        else:
            print(f"Error: Directory not found: {mc_dir}")

    else:
        print("\n--- MODE: Single Simulation Run ---")
        single_bin = os.path.join(script_dir, "simulation_data.bin")
        single_parquet = os.path.join(script_dir, "simulation_data.parquet")
        convert_bin_to_parquet(single_bin, single_parquet, record_header)