# Museum Security Robot

An AI-assisted autonomous museum patrol project using ROS 2, Nav2, SLAM Toolbox, and Gazebo.

The robot accepts natural language security commands, uses a Gemini model to infer a target exhibit area, and then sends a navigation goal to Nav2.

## Project Highlights

- AI intent parsing from free-form user commands
- Autonomous navigation with Nav2 (path planning and control)
- TF fix node that publishes odom -> base_link from Odometry
- Custom Gazebo museum world
- Included SLAM and map metadata configuration
- RViz configuration for visualization

## Repository Contents

- `museum_ai_navigator.py`: Main AI command interface and Nav2 goal dispatcher
- `odom_tf_broadcaster.py`: ROS 2 node publishing dynamic TF (`odom` -> `base_link`)
- `museum.sdf`: Gazebo SDF world file for the museum environment
- `museum_map.pgm` + `museum_map.yaml`: Saved occupancy grid map and metadata
- `museum_nav2_params.yaml`: Nav2 stack parameters (AMCL, planner, controller, costmaps, behavior server, etc.)
- `museum_slam.yaml`: SLAM Toolbox mapping parameters
- `museum.rviz`: RViz display configuration
- `LICENSE`: MIT license

## High-Level Flow

1. Start simulation and robot stack.
2. Start Nav2 with `museum_nav2_params.yaml` and a map (or SLAM mode with `museum_slam.yaml`).
3. Run `odom_tf_broadcaster.py` to provide dynamic odom TF.
4. Run `museum_ai_navigator.py`.
5. Enter natural-language commands (for example, "check the statue" or "someone suspicious entered the building").
6. The script sends the selected location as a `PoseStamped` goal via Nav2.

## Museum Target Locations

Defined in `museum_ai_navigator.py`:

- `lobby`: x=0.474, y=1.380
- `dinosaur_exhibit`: x=2.800, y=2.619
- `renaissance_statue`: x=0.513, y=-1.653
- `ai_innovation_wing`: x=-1.573, y=0.491
- `cafeteria`: x=-0.477, y=2.502

## Prerequisites

- macOS/Linux development environment
- ROS 2 installation
- Navigation2 (Nav2)
- SLAM Toolbox
- Gazebo with SDF support
- Python 3 packages:
	- `rclpy`
	- `nav2_simple_commander`
	- `google-genai`

You also need a Gemini API key:

- Set environment variable `GEMINI_API_KEY`

Example:

```bash
export GEMINI_API_KEY="YOUR_API_KEY"
```

## Typical Run Sequence

Use separate terminals after sourcing your ROS 2 environment.

1. Start Gazebo with the museum world.

```bash
gz sim museum.sdf
```

2. Bring up Nav2 with map and parameters.

```bash
ros2 launch nav2_bringup navigation_launch.py \
	use_sim_time:=true \
	map:=$PWD/museum_map.yaml \
	params_file:=$PWD/museum_nav2_params.yaml
```

3. Run the TF broadcaster node.

```bash
python3 odom_tf_broadcaster.py
```

4. Run the AI navigator.

```bash
python3 museum_ai_navigator.py
```

5. Optionally open RViz configuration.

```bash
rviz2 -d museum.rviz
```

## Notes

- If AI-inferred navigation seems off, adjust the location coordinates in `MUSEUM_LOCATIONS` inside `museum_ai_navigator.py` to match your map frame.
- If `GEMINI_API_KEY` is not set, the AI navigator exits with a runtime error.
- If the model returns an unexpected label, the script safely defaults to `lobby`.

## License

This project is licensed under the MIT License. See `LICENSE`.

