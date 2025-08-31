# How to use

1. change path names from agrim -> your_name
2. make sure to have models folder in .gazebo
3. build ws
4. this cmd for launching the simulation <br>
```bash
ros2 launch Simpli_description gazebo_world.launch.py
```
5. check if controllers are loaded
```bash
ros2 control list_controller
```
if no controller then do
```bash
ros2 run controller_manager spawner joint_state_broadcaster
```
```bash
ros2 run controller_manager spawner ackermann_steering_controller
```

6. launch joystick node, change the mapping as per your liking
```bash
ros2 launch system_control joys_control.launch.py
```

# Issues
- the issue is that this uses ackermann controller which only moves the rear wheels and uses frontwheels for direction, when the rover spawns some wheels are not at home so even though we give forward cmd the rover moves diagonally

## Solutions
use the four_ws.yaml for four wheel controller but you have to build it from source as that controller is not available in apt repositories
