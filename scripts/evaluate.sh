#!/usr/bin/env bash
# Evaluate the recorded trajectories against TUM ground truth with evo.
set -euo pipefail

SEQUENCE=${SEQUENCE:-rgbd_dataset_freiburg1_xyz}
GT=/data/${SEQUENCE}/groundtruth.txt
R=/results
export MPLBACKEND=Agg

for est in vo slam; do
  echo "== APE (${est}) =="
  evo_ape tum "$GT" "$R/${est}_traj.txt" --align --no_warnings \
      --plot_mode xy --save_plot "$R/ape_${est}.png" \
      --save_results "$R/ape_${est}.zip" | tee "$R/ape_${est}.txt"
done

evo_traj tum "$R/vo_traj.txt" "$R/slam_traj.txt" --ref "$GT" --align --no_warnings \
    --plot_mode xy --save_plot "$R/trajectories.png"

echo "== Summary =="
evo_res "$R/ape_vo.zip" "$R/ape_slam.zip" --use_filenames --no_warnings | tee "$R/summary.txt"
