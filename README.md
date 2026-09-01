A simple trajectory planner for turtlesim using LLM

[ User ] -------------> ( Text prompt ) 
                            |  
                            v  /llm_cmd
           +---------------------------------------+
           |       ROS2 node (turtlesim_llm)       |
           |                                       |
           |  +---------------------------------+  |
           |  | LLM Interface (foundry-local)   |  |
           |  | w/system Prompt (Generate JSON) |  |
           |  +---------------------------------+  |
           |                |                      |
           |                v                      |
           |    +--------------------------+       |
           |    | JSON Parser / Interpreter|       |
           |    +--------------------------+       |
           |                                       |
           |                                       |
           +---------------------------------------+
                            |           
                            |   /turtlesim_waypoint_follower/follow_cmd (ROS2 Action)
                            v
           +---------------------------------------+
           |       ROS2 node (waypoint_follower)   |
           |                                       |
           |    +-------------------------------+  |
           |    |         Control Loop          |  |<--- /turtle1/pose (turtlesim)
           |    +-------------------------------+  |           
           |                                       |
           +--+------------------------------------+
                           |
                           v  /turtle1/cmd_vel
                     [ turtlesim ]


**_Install_**

Compilation requires Rust and ros-rust to be installed. 
More at https://github.com/ros2-rust/ros2_rust


**_Simulation_**

Generally each command should be run in separate terminal tab. Please remember to run source command. 

_turtlesim_
``` bash
ros2 run turtlesim turtlesim_node
```
_turtlesim_llm_
``` bash
ros2 run MecHaRo-Lab_turtlesim_llm_rs turtlesim_llm
```
_waypoint_follower_
``` bash
ros2 run MecHaRo-Lab_turtlesim_llm_rs waypoint_follower
```
_User - example text prompt_
``` bash
ros2 topic pub --once /llm_cmd std_msgs/msg/String "data: 'draw a rectangle'"
```

