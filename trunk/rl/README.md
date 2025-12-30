### Create a symbolic link from this path to the ROS workspace

```bash
ln -s ~/intel-edge-ai-sw-8/2601_4th_proj_dahyeon/trunk/rl/proj_wifi_heatmap_rl ~/turtlebot3_ws/src/proj_wifi_heatmap_rl
```

### Build and Run

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

```bash
cd ~/turtlebot3_ws
python3 -m proj_wifi_heatmap_rl.train_dqn
python3 -m proj_wifi_heatmap_rl.train_dqn --resume runs_dqn/dqn_ep0800.pt --more-episodes 200
python3 -m proj_wifi_heatmap_rl.train_dqn --resume runs_dqn/dqn_ep0800.pt --total-episodes 1000
python3 -m proj_wifi_heatmap_rl.train_dqn --resume runs_dqn/dqn_ep0800.pt --more-episodes 200 --reset-optim
```

```bash
python3 -m proj_wifi_heatmap_rl.test_dqn_policy --ckpt runs_dqn/dqn_ep0800.pt
python3 -m proj_wifi_heatmap_rl.test_dqn_policy --ckpt runs_dqn/dqn_ep0800.pt --episodes 1 --sleep 0.05
```

---

### Publish Test Topic

```bash
# once
ros2 topic pub --once /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    frame_id: map
  },
  pose: {
    pose: {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}"

# 2Hz
ros2 topic pub /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    frame_id: map
  },
  pose: {
    pose: {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  }
}" -r 2
```
