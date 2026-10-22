# ArUco docking

This repository uses OpenCV's ArUco tracker for the fiducial detection part of docking. The upstream [Automatic Addison Jazzy docking tutorial](https://automaticaddison.com/autonomous-docking-with-apriltags-using-nav2-ros-2-jazzy/) demonstrates AprilTags; this project keeps the Nav2 docking integration and uses an ArUco marker.

## Configuration

- Tracker parameters: `yahboom_rosmaster_docking/config/aruco_tracker.yaml`
- Dock pose database: `yahboom_rosmaster_docking/config/dock_database.yaml`
- Detector and pose-adapter launch: `yahboom_rosmaster_docking/launch/aruco_dock_pose_publisher.launch.py`
- Gazebo marker model: `yahboom_rosmaster_gazebo/models/ArUco4X4_50_00000`

The detector uses OpenCV's `4X4_50` dictionary, marker ID 0, and a 0.075 m coded marker edge. The simulation texture includes a one-cell white margin, making the complete target 0.10 m wide. Print marker ID 0 from `4X4_50` at the matching scale. When changing the dictionary, ID, or dimensions, update the tracker parameters, launch `marker_id`, and printed/simulated target together.

The camera must publish calibrated `color/image_raw` and `color/camera_info` topics under the camera namespace. The launch rectifies the image to `color/image_rect`; the tracker publishes marker poses on TF as `marker_<id>`. The adapter republishes the selected camera-to-marker pose on `/detected_dock_pose`, preserving the Nav2 docking interface.

## Launch the detector

With the Jazzy workspace sourced and dependencies installed:

```bash
ros2 launch yahboom_rosmaster_docking aruco_dock_pose_publisher.launch.py
```

The integrated bringup launch also starts the detector. Inspect the camera topics, `/tf`, and `/detected_dock_pose` to confirm that the marker is detected and its pose is expressed in the camera optical frame.

## Tune the dock

Update the dock pose database and docking server values in the Nav2 parameter file together. The detected pose describes the marker relative to the camera; the dock configuration supplies the desired approach and staging offset. Start at low approach speeds and validate marker orientation and staging pose in simulation before testing on the physical robot.

## Install dependency

`aruco_opencv` is released for ROS 2 Jazzy and is installed by rosdep from the docking package manifest. If rosdep cannot resolve it in your environment:

```bash
sudo apt install ros-jazzy-aruco-opencv
```
