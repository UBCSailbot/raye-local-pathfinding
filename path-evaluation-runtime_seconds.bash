#!/usr/bin/env bash

# Hyperparameter grid search based on total runtime
for total_runtime in 1.0
do
    for runtime_seconds in 0.1 0.125 0.25 0.5
    do
        if (( $(echo "$runtime_seconds > $total_runtime" |bc -l) ))
        then
            continue
        fi
        num_runs=$(echo "$total_runtime / $runtime_seconds" | bc)
        for grid_n in 10 15 20 25
        do
            for planner_type in "lazyprmstar"
            do
                for global_wind_direction_degrees in 0 30 45 90
                do
                    for num_ais_ships in 5 20 50
                    do
                        for random_seed in "1" "310"
                        do
                        echo "timeout $(echo "scale=4; $num_runs*$runtime_seconds+30" | bc) roslaunch local_pathfinding path_evaluator.launch num_runs:=$num_runs runtime_seconds:=$runtime_seconds state_sampler:="grid" grid_n:=$grid_n planner_type:=$planner_type global_wind_direction_degrees:=$global_wind_direction_degrees num_ais_ships:=$num_ais_ships random_seed:=$random_seed"

                        timeout $(echo "scale=4; $num_runs*$runtime_seconds+30" | bc) roslaunch local_pathfinding path_evaluator.launch num_runs:=$num_runs runtime_seconds:=$runtime_seconds state_sampler:="grid" grid_n:=$grid_n planner_type:=$planner_type global_wind_direction_degrees:=$global_wind_direction_degrees num_ais_ships:=$num_ais_ships random_seed:=$random_seed || :
                        done
                    done
                done
            done
        done
    done
done

