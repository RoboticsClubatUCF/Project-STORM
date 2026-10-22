# ROSMASTER project Bash shortcuts

The repository provides a small set of Bash aliases for the common ROS 2 Jazzy workspace, build, launch, and diagnostic commands.

## Load the shortcuts

The default workspace path is `~/ros2_ws`. Clone this repository into `~/ros2_ws/src/yahboom_rosmaster`, then load the shortcuts in each new Bash terminal:

```bash
export ROSMASTER_WS="$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$ROSMASTER_WS/src/yahboom_rosmaster/scripts/rosmaster-aliases.bash"
sb
```

`sb` sources the Jazzy environment and the workspace overlay. If your workspace uses a different path, set `ROSMASTER_WS` before sourcing the alias file. For example:

```bash
export ROSMASTER_WS="$HOME/robot_ws"
source "$ROSMASTER_WS/src/yahboom_rosmaster/scripts/rosmaster-aliases.bash"
sb
```

Aliases are a Bash feature. These shortcuts are loaded in the current terminal only; add the `source .../rosmaster-aliases.bash` line to `~/.bashrc` if you want them in every new Bash terminal. Keep the workspace path export before that line.

## Workspace and build commands

| Alias | Command |
| --- | --- |
| `cws` | Change to the workspace root |
| `sj` | Source `/opt/ros/jazzy/setup.bash` |
| `sw` | Source `$ROSMASTER_WS/install/setup.bash` |
| `sb` | Source Jazzy and the workspace overlay |
| `cb` | Build the whole workspace with `--symlink-install` |
| `cbu <package>` | Build a package and its dependencies with `--packages-up-to` |
| `ct` | Run `colcon test` for the workspace |
| `ctpkg <package>` | Run tests for selected package(s) |
| `ctresults` | Print the workspace test results |

Examples:

```bash
cws
cb
cbu yahboom_rosmaster_navigation
ctpkg yahboom_rosmaster_docking
ctresults
```

The full `ct` command runs every package's declared tests, including repository-wide lint checks. Some existing formatting checks currently fail; the Jazzy CI workflow instead gates on a successful full build, launch argument checks, and the docking startup smoke test.

## Robot launch commands

| Alias | Command |
| --- | --- |
| `simx3` | Launch the ROSMASTER X3 Gazebo simulation |
| `navx3` | Launch Gazebo, localization, Nav2, and docking with the cafe map |
| `dockaruco` | Launch the ArUco tracker and dock-pose publisher |

Examples:

```bash
simx3
navx3
dockaruco
dockaruco use_sim_time:=false
```

The `navx3` alias uses the installed `cafe_world_map.yaml` and `cafe.world`. Append launch arguments to override values, for example `navx3 slam:=True`.

## Diagnostics

| Alias | Command |
| --- | --- |
| `rnodes` | List running ROS nodes |
| `rtopics` | List visible ROS topics |
| `rscaninfo` | Show the type and publishers/subscribers for `/scan` |
| `rtf` | Generate a TF frame report with `tf2_tools view_frames` |
| `rcontrollers` | List ros2_control controllers |

These commands inspect a running ROS graph; start the simulation or robot bringup first.

## Verify the aliases

From the repository root, run:

```bash
bash scripts/test_rosmaster_aliases.bash
```

The check validates that every documented alias is defined, that key aliases point to the expected project commands, and that `cws` changes to `ROSMASTER_WS`. GitHub Actions also uses the aliases for its Jazzy build, launch-description checks, and ArUco startup smoke test.
