```markdown
# AI-Middleware Instructions

## Overview
This guide explains how to run the AI-Middleware, start the `turtlesim_node`, and monitor a background skill. The background skill manager will enable the pen when the turtle is within a designated square and disable it when the turtle exits this area.

## Steps

1. **Run the AI-Middleware**

   Run the AI-Middleware with this directory as a parameter:
   ```sh
   # Replace <directory> with the path to your <AI-Middleware> directory
   python3 ai_middleware_generator.py ~/ros2_ws <AI-Middleware>/Examples/Example1_monitoring turtlesim/turtlesim_node 
   
   #On my machine it was:
   #python3 ai_middleware_generator.py ~/ros_ws /home/lab/Projects/AI-Middleware-ROS2/Examples/Example1_monitoring turtlesim/turtlesim_node
   ```

2. **Start the Turtlesim Teleoperation**

   In a new terminal, run the turtlesim teleoperation node to control the turtle:
   ```sh
   ros2 run turtlesim turtle_teleop_key
   ```

3. **Move the Turtle**

   Use the teleop keys to move the turtle. The background skill manager will monitor the turtle's position. When the turtle enters a designated square, the skill manager will enable the pen. When the turtle exits this area, the pen will be disabled.
```

Replace `<directory>` with the actual path to your directory in the AI-Middleware command. This format ensures the instructions are clear and easily understood within a GitHub README file.
