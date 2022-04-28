#!/usr/bin/env bash

# Syncs the runs in a wandb latlon plot directory created by wandb_log_latlon_plots.py
# usage: ./wandb_sync_latlon_plots.bash <absolute path to latlon plots directory>
# example: ./wandb_sync_latlon_plots.bash ~/.ros/latlon_plots/Apr_28-14_25_40
for d in $1/wandb/*/ ; do
    if [[ $d != *"latest-run/" ]]; then
        wandb sync $d
    fi
done
