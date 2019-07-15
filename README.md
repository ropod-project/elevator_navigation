# elevator_navigation

## Summary

A ROS component that manages action calls for elevator navigation. The component subscribes to four different action types:

* `WAIT_FOR_ELEVATOR`
* `ENTER_ELEVATOR`
* `RIDE_ELEVATOR`
* `EXIT_ELEVATOR`

The actions are exposed through a single action server.

## Launch file parameters

The component expects several parameters to be made available to the ROS parameter server:
* `mn_nav_topic: str` -- name of a topic for sending maneuver navigation goals (default `/route_navigation/goal`)
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
