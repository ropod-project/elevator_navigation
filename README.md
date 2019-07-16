# elevator_navigation

## Summary

A ROS component that manages action calls for elevator navigation. The component subscribes to four different action types:

1. `WAIT_FOR_ELEVATOR`
2. `ENTER_ELEVATOR`
3. `RIDE_ELEVATOR`
4. `EXIT_ELEVATOR`

These actions are defined in [this PDDL domain](https://github.com/ropod-project/task-planner/blob/master/config/task_domains/agaplesion/hospital_transportation.pddl).

The actions are exposed through a single action server, such that action calls are assumed to be made in the above order. The split into four actions is made for the purposes of simplified fault tolerance, as each individual step of the sequence can be triggered and then monitored independently of the others.

The component is embedded into a [fault-tolerant state machine](https://github.com/ropod-project/ftsm) and has [this specification](https://github.com/ropod-project/component-monitoring/blob/master/component_monitoring/component_config/robot/software/elevator_navigation.yaml).

## Dependencies

The elevator navigation depends on the following other components:
* [`ropod_rod_msgs`](https://git.ropod.org/ropod/communication/ropod_ros_msgs): Contains message and action definitions used within the component
* [`maneuver_navigation`](https://github.com/ropod-project/ros-structured-nav): Handles navigation requests
* [`world_model_mediator`](https://git.ropod.org/ropod/wm/ropod_wm_mediator): Handles world model queries
* [`door_status_detection`](https://git.ropod.org/ropod/navigation/door_status_detection): Checks whether an elevator door is open or closed
* [`floor_detection`](https://git.ropod.org/ropod/navigation/floor_detection): Tracks the current floor of the robot
* [`map_switcher`](https://git.ropod.org/ropod/navigation/map_switcher): Handles requests for loading new navigation maps after floor changes

## Launch file parameters

The component expects several parameters to be made available to the ROS parameter server:
* `mn_nav_topic: str` -- name of a topic for sending maneuver navigation goals (default `/route_navigation/goal`)
* `mn_nav_cancel_topic: str` -- name of a topic for cancelling maneuver navigation goals (default `/route_navigation/cancel`)
* `init_pose_topic: str` -- name of a topic for initialising the pose of a robot (default `/initialpose`)
* `localisation_topic: str` -- name of a topic on which localisation pose estimates are published (default `/amcl_pose`)
* `elevator_nav_server_name: str` -- name of the action server exposed by the component (default `/ropod/take_elevator`)
* `elevator_waypoints_server: str` -- name of an action server that responds to elevator waypoint queries (default `/get_elevator_waypoints`)
* `topology_node_server: str` -- name of a server that responds to queries regarding area topologies (default `/get_topology_node`)
* `door_status_detection_server: str` -- name of a service for checking whether a door is open or closed (default `/get_door_status`)
* `floor_detection_server: str` -- name of a service that returns the currrent floor of a robot (default `/floor_detection_server`)
* `map_switcher_server: str` -- name of a server for loading new environment maps (default `/map_switcher/change_map`)

* `go_to_elevator_timeout: float` -- timeout (in seconds) for the action of going to a waypoint where a robot can wait for an elevator (default `120`)
* `wait_for_elevator_timeout: float` -- timeout (in seconds) for the action of waiting for an elevator (default `120`)
* `enter_elevator_timeout: float` -- timeout (in seconds) for the action of entering an elevator (default `120`)
* `ride_elevator_timeout: float` -- timeout (in seconds) for the action of riding an elevator (default `600`)
* `exit_elevator_timeout: float` -- timeout (in seconds) for the action of exiting an elevator (default `120`)

* `pos_tolerance_m: float` -- distance tolerance (in meters) for navigation waypoints (default `0.3`)
* `orientation_tolerance_deg: float` -- orientation tolerance (in degrees) for navigation waypoints (default `20`)
