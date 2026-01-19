## Data Pipeline

main.cpp runs in /src -> 

main.cpp outputs simulation_data.bin into /data using format described in RecordData.h -> 

BinToParquet gets called in main.cpp -> 

BinToParquet outputs simulation_data.parquet in /data using simulation_data.bin ->

SimVis tools in /visualization use simulation_data.parquet in /data to plot and animate
