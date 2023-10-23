import pandas as pd
import numpy as np
import seaborn as sns
from matplotlib import pyplot as plt
import math
import os
import copy

# Solution Fitness History
def plotFitness(filepath):
    solution_hist = pd.DataFrame()
    run_count = 0
    first_eval=0
    while(os.path.isfile(f"{filepath}/run_{run_count}/solution_history")):
        fitness = pd.read_csv(f"{filepath}/run_{run_count}/solution_history")
        fitness.rename(columns=lambda x: x.strip(),inplace=True)
        
        if(run_count == 0): first_eval = fitness['evaluation']
        # fitness['evaluation']=fitness['evaluation']
        fitness['evaluation']=first_eval
        fitness['run']=run_count
        fitness['generation']=fitness.index
        fitness['solution_fitness'] = fitness['solution_fitness']

        solution_hist = pd.concat([solution_hist,fitness],ignore_index=True)
        run_count += 1

    solution_hist.rename(columns=lambda x: x.strip(),inplace=True)
    print(solution_hist)

    ## Learning Curves
    plt.figure()
    sns.lineplot(data=solution_hist, x="evaluation", y='solution_fitness')

    plt.xscale("log")
    plt.title(f"Learning Curves")
    plt.ylabel("Fitness (m/s)")
    plt.xlabel("Evaluations")
    plt.savefig(f"{filepath}/learning_curve.png")

def plotComparedFitness(filepaths, stratagies):
    solution_hist = pd.DataFrame()
    strat = 0
    for filepath in filepaths:
        run_count = 0
        first_eval=0
        while(os.path.isfile(f"{filepath}/run_{run_count}/solution_history")):
            fitness = pd.read_csv(f"{filepath}/run_{run_count}/solution_history")
            fitness.rename(columns=lambda x: x.strip(),inplace=True)
            
            if(run_count == 0): first_eval = fitness['evaluation']
            # fitness['evaluation']=fitness['evaluation']
            fitness['evaluation']=first_eval
            fitness['run']=run_count
            fitness['strat'] = stratagies[strat]
            fitness['generation']=fitness.index
            fitness['solution_fitness'] = fitness['solution_fitness']

            solution_hist = pd.concat([solution_hist,fitness],ignore_index=True)
            run_count += 1
        strat += 1

    solution_hist.rename(columns=lambda x: x.strip(),inplace=True)
    print(solution_hist)

    ## Learning Curves
    plt.figure()
    sns.lineplot(data=solution_hist, x="evaluation", y='solution_fitness', hue='strat')

    plt.xscale("log")
    plt.title(f"Learning Curves")
    plt.ylabel("Fitness (m/s)")
    plt.xlabel("Evaluations")
    plt.savefig(f"compared_learning_curves.png")

def plotDiversity(filepath):
    div_hist = pd.DataFrame()    
    run_count = 0
    while(os.path.isfile(f"{filepath}/run_{run_count}/diversity_history")):
        diversity = pd.read_csv(f"{filepath}/run_{run_count}/diversity_history")
        diversity.rename(columns=lambda x: x.strip(),inplace=True)

        organisms = diversity.columns.to_list()
        diversity= pd.melt(diversity, id_vars=["evaluation"], value_vars=organisms)

        diversity["run"] = run_count
        div_hist = pd.concat([div_hist,diversity],ignore_index=True)

        run_count += 1

    run_rows = lambda df: df["run"]==1
    
    # run_diversity = div_hist
    run_diversity = div_hist[run_rows]
    max_dist = run_diversity.groupby(["evaluation"])['value'].max()

    plt.figure()
    sns.lineplot(data=max_dist)
    sns.scatterplot(x="evaluation",y="value",
                data=run_diversity)

    plt.xscale("log")
    plt.ylim([0,10200])
    plt.title(f"Diversity")
    plt.ylabel("distance")
    plt.xlabel("Evaluations")
    plt.savefig(f"{filepath}/div_plot.png")

if __name__ == "__main__":
    # TODO: ingest filepath so we can call from evodevo
    filepath1 = "/home/dawson/evo-devo/z_results/2023-10-09-131429"
    filepath2 = "/home/dawson/evo-devo/z_results/2023-10-09-114918"
    plotComparedFitness([filepath1, filepath2], ["Evo", "EvoDevo"])
    plotDiversity(filepath2)
    plotFitness(filepath2)