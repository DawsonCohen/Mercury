import pandas as pd
import numpy as np
import seaborn as sns
from matplotlib import pyplot as plt
import math
import os
import copy

# Solution Fitness History
def plotTrace(filepath):
    trace = pd.DataFrame()
    filename = f"{filepath}/sim_trace_0.csv"
    if(os.path.isfile(filename)):
        trace = pd.read_csv(filename)
        trace.rename(columns=lambda x: x.strip(),inplace=True)

    filename = f"{filepath}/sim_trace_1.csv"
    if(os.path.isfile(filename)):
        trace2 = pd.read_csv(filename)
        trace2.rename(columns=lambda x: x.strip(),inplace=True)
        
    print(trace)


    ## Learning Curves
    plt.figure()

    g = sns.lineplot(data=trace, x="time", y='x', hue='id')
    plt.legend([],[], frameon=False)

    plt.title(f"Simulation Trace")

    plt.xlabel("Simulation Time (s)")
    plt.ylabel("X Position")
    plt.savefig(f"{filepath}/trace_all.png")

    trace["diff"] = trace["x"] - trace2["x"];
    plt.figure()

    g = sns.lineplot(data=trace, x="time", y='diff', hue='id')
    plt.legend([],[], frameon=False)

    plt.title(f"Simulation Trace Difference")

    plt.xlabel("Simulation Time (s)")
    plt.ylabel("X Position Difference")
    plt.savefig(f"{filepath}/trace_all_diff.png")


if __name__ == "__main__":
    # TODO: ingest filepath so we can call from tests
    filepath = "/path/to/z_results"
    plotTrace(filepath)