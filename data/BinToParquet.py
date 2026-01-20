#
# ~ BinToParquet.py ~
# Owns: 
# - Simulation_data.bin formatting
# *Ultized Claude for general code functions as converting file types is not a main objective of this project

import numpy as np
import pandas as pd
import re
import os

def parse_cpp_struct(header_path):
    dtype_list = []
    column_map = {} 

    with open(header_path, 'r') as f:
        content = f.read()

    match = re.search(r'struct\s+RecordData\s*\{(.*?)\};', content, re.DOTALL)
    if not match:
        raise ValueError("Could not find 'struct LogRecord'")
    
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

def convert_bin_to_parquet(bin_file, parquet_file, header_file):
    print(f"Parsing {header_file}...")
    dt, col_map = parse_cpp_struct(header_file)
    
    print("Reading binary...")
    data = np.fromfile(bin_file, dtype=dt)
    
    print("Building DataFrame...")
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
    print(f"Saved {len(df)} rows to {parquet_file}")

    # verify the parquet file was created and delete the bin file
    if os.path.exists(parquet_file) and os.path.getsize(parquet_file) > 0:
        os.remove(bin_file)
        print(f"Deleted {bin_file}")
    else:
        print(f"WARNING: Parquet file verification failed, keeping {bin_file}")

if __name__ == "__main__":

    script_dir = os.path.dirname(os.path.abspath(__file__))

    bin_file = os.path.join(script_dir, "simulation_data.bin")
    parquet_file = os.path.join(script_dir, "simulation_data.parquet")
    header_file = os.path.join(script_dir, "RecordData.h")

    # run conversion
    convert_bin_to_parquet(bin_file, parquet_file, header_file)