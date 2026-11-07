# RGB-D SLAM in ROS 2, fully Dockerized

RTAB-Map RGB-D SLAM running in ROS 2 Humble inside Docker, fed by a custom ROS 2 node that replays
TUM RGB-D sequences with their original timestamps, and evaluated against motion-capture ground truth
with [evo](https://github.com/MichaelGrupp/evo). One command to run, one command to evaluate.

![demo](docs/demo.gif)

## Results

Absolute trajectory error (ATE, translation RMSE after SE(3) Umeyama alignment), TUM RGB-D:

| Sequence | Frames tracked | Visual odometry | RTAB-Map SLAM | Improvement |
|---|---|---|---|---|
| fr1/xyz  | 789 / 789 | 3.94 cm | **1.77 cm** | −55 % |
| fr1/desk | 552 / 554 | 7.74 cm | **6.46 cm** | −17 % |

![trajectories on fr1/desk](docs/trajectories_desk.png)

**Reading:** frame-to-frame odometry accumulates drift; RTAB-Map's graph optimization corrects it.
The gain is largest on fr1/xyz, where the camera stays over the same desk and every new frame can be
matched against the map. On fr1/desk the motion is faster and the scene changes more, so there is less
to correct and the remaining error is dominated by the odometry itself (see roadmap: calibrated
intrinsics with distortion and a higher map update rate are the first things to try).

## Pipeline

```
TUM sequence (PNG) ─► tum_publisher (ROS 2 node) ─► /camera/rgb, /camera/depth, /camera_info, /clock
                                                         │
                                                         ▼
                                    RTAB-Map: RGB-D visual odometry ─► graph SLAM + loop closure
                                                         │
                              trajectory_recorder ◄──────┤──────► Foxglove (3D map, tf, path)
                                     │
                                     ▼
                        vo_traj.txt / slam_traj.txt ─► evo (ATE vs ground truth)
```

## Quick start

Requirements: Docker (Docker Desktop on Windows/macOS). Nothing else on the host.

```bash
# 1. Get a sequence (~0.4 GB)
cd data
curl -L -o fr1_xyz.tgz https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
tar -xzf fr1_xyz.tgz && cd ..

# 2. Build and run SLAM (stops by itself at the end of the sequence)
docker compose up --build slam

# 3. Evaluate (plots and tables land in results/)
docker compose run --rm eval
```

Other sequence: `SEQUENCE=rgbd_dataset_freiburg1_desk docker compose up slam`
(PowerShell: `$env:SEQUENCE="rgbd_dataset_freiburg1_desk"` first).

Live view: open [Foxglove](https://app.foxglove.dev), connect to `ws://localhost:8765`, add a 3D panel
with display frame `map`.

## Engineering notes

Getting large images through ROS 2 reliably took more than wiring nodes together:

- **Custom dataset player instead of the official rosbags.** The TUM bags are ROS 1, use frame ids with a
  leading `/` (rejected by tf2 in ROS 2) and publish mocap ground truth on `/tf`, which conflicts with the
  SLAM tf tree. The player replays the PNGs with dataset time on `/clock`, so every estimate keeps the
  timestamp of its ground-truth pose.
- **Middleware tuned for images.** Fast DDS's default shared-memory segment (512 KB) is smaller than one
  RGB (0.9 MB) or depth (1.2 MB) frame, so frames fell back to fragmented UDP and RTAB-Map's exact-time
  synchronizer received ~1 % of RGB/depth/info triplets. A Fast DDS profile with 64 MB segments
  (`config/fastdds.xml`) plus a 2 GB container `/dev/shm` brought that to 100 %.
- **rclpy fast path.** Assigning image bytes to `Image.data` makes rclpy validate every byte in Python;
  passing an `array.array` instead raised the player from ~3 to ~10+ fps.
- **Two trajectories from one run.** `vo` is pure odometry (`odom → camera_link`); `slam` composes
  RTAB-Map's `map → odom` correction, so the table isolates what the SLAM back end contributes.

## Repository layout

```
Dockerfile, docker-compose.yml   ROS 2 Humble + RTAB-Map + Foxglove bridge + evo
config/fastdds.xml               middleware profile for large images
launch/tum_slam.launch.py        player, RTAB-Map, recorder, Foxglove, static tf
scripts/tum_publisher.py         TUM sequence → ROS 2 topics
scripts/trajectory_recorder.py   odometry / SLAM poses → TUM-format trajectories
scripts/evaluate.sh              evo APE, plots and summary
```

## Roadmap

- [x] Stage 1: Dockerized ROS 2 + RTAB-Map + evo evaluation on TUM RGB-D
- [ ] Use the calibrated fr1 intrinsics and distortion; tune the map update rate
- [ ] Stage 2: replace RTAB-Map's odometry with my own stereo visual odometry node (EuRoC)
- [ ] Stage 3: own back end: keyframes, GTSAM pose graph, bag-of-words loop closure
