#!/usr/bin/env bash

# Syncs the runs in a wandb directory
# usage: ./wandb_sync_dir.bash <absolute path to latlon plots directory>
# example: ./wandb_sync_dir.bash ~/.ros/latlon_plots/latest-dir
# example: ./wandb_sync_dir.bash ~/.ros/stats/latest-dir
for d in $1/wandb/*/ ; do
    if [[ $d == *"offline-run-"* ]]; then
        wandb sync $d
    fi
done
