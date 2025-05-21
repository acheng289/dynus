# Development Notes
- Running on WSL2, Ubuntu-22.40 and Docker
- On WSL2 Terminal, rqt works as intended but inside container no go
- Trying to install VcXsrv and set it up first
- set .wslconfig to have mirrored network
- dyn_obstacles.launch.py spawns multiple obstacles, can refer to it to see how we might spawn more than 1 agent. Note robot_state_publisher and spawn_entity
- if installing new stuff try to put at the end of docker file for the cache to speed up build