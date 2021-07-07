#!/usr/bin/env bash

# Hyperparameter grid search
for num_runs in 1 2 4 8
do
for runtime_seconds in 0.5 1.0 2.0 5.0
do
for state_sampler in "grid"
do
for grid_n in 10
do
for planner_type in "prmstar" "rrtsharp" "rrtxstatic"
do
for global_wind_direction_degrees in 45
do
for num_ais_ships in 20
do
for random_seed in "11" "1310" "111"
do
total_runtime=$(echo "scale=4; $num_runs*$runtime_seconds" | bc)
# Don't test with very long runtime
if (( $(echo "$total_runtime > 10.0" | bc -l) )); then
  continue
fi
timeout_=$(echo "scale=4; $total_runtime+30" | bc)
echo "timeout $timeout_ roslaunch local_pathfinding path_evaluator.launch num_runs:=$num_runs runtime_seconds:=$runtime_seconds state_sampler:=$state_sampler grid_n:=$grid_n planner_type:=$planner_type global_wind_direction_degrees:=$global_wind_direction_degrees num_ais_ships:=$num_ais_ships random_seed:=$random_seed"
timeout $timeout_ roslaunch local_pathfinding path_evaluator.launch num_runs:=$num_runs runtime_seconds:=$runtime_seconds state_sampler:=$state_sampler grid_n:=$grid_n planner_type:=$planner_type global_wind_direction_degrees:=$global_wind_direction_degrees num_ais_ships:=$num_ais_ships random_seed:=$random_seed || :
done
done
done
done
done
done
done
done
