## Important!!!
### Follow these steps to avoid any stupid merging conflicts in the end:
1. Pull from main
2. Switch to your branch (1 branch per person pls)
3. Write your code (do not touch others code without a reason)
4. Fully test your code before commiting
5. Never merge your own pull request (unless its something obvious)

# To Do List

### 3. Add it to the TurtleBot3 model in Gazebo

### 4. Start writing code
#### 4.1. Block recognition (Duncan)
#### 4.2. Arena discovery (Duncan)

### 5.Add it to the TurtleBot3 model in Gazebo


# Tutorial

## Build (DO THIS FIRST, ESPECIALLY IF YOU'VE MADE CHANGES!!!)
```
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
```
![Expected output](/images/expected_output.png)

Remember to source after building!
```
source install/setup.bash
```

## Launching Arena World
```
cd ~/ros2_ws
ros2 launch turtlebot3_task turtlebot3_world.py \
	marker_north:=7 marker_south:=42 \
	marker_east:=23 marker_west:=0 \
	red_marker:=<AruCo marker> blue_marker:=<AruCo marker>
```
Available AruCo markers are 0 (W), 7 (N), 23 (E), 42 (S).  

## Checking Blue and Red Markers

After launching the world, if you run 
```
ros2 param get /competition_logic (blue/red_goal_id)
```
it will tell you which marker has been assigned that colour 

## Running Commands on the Turtlebot
First ssh into the robot
```
ssh tb@1100.127.105.110
```
### Camera and LiDAR
To run the camera, and make it run smoothly
```
ros2 launch turtlebot3_bringup camera.launch.py format:=YUYV width:=320 height:=240
```
or
```
ros2 launch turtlebot3_bringup camera.launch.py format:=YUYV width:=160 height:=120
```
, then to run the lidar sensor
```
ros2 launch turtlebot3_bringup robot.launch.py
```

### Executables
To run our file, do it on the VM
```
ros2 run turtlebot3_task inshallah.py
```
## Note:
All executable code goes to the turtlebot3_task folder, any worlds into worlds folder, models into models, etc. (look in the turtlebot3_gazebo folder for an example).
