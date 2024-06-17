Well I was going to write a brief documentation about this but unfortunately it is bedtime

    ros2 launch my_ros_bringup turtlebot.launch.xml
    ros2 action send_goal /move_turtle my_ros_interface/action/MoveTurtle "{linear_vel_x: 0.5, angular_vel_z: 0.7, duration_sec: 5}"
    
but these 2 commands should give some example of the functionality of the code for the areas below

# Action
# Lifecycle Nodes
# Executor
# Component/Composition
