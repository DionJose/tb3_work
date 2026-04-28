## Important!!!
### Follow these steps to avoid any stupid merging conflicts in the end:
1. Pull from main
2. Switch to your branch (1 branch per person pls)
3. Write your code (do not touch others code without a reason)
4. Fully test your code before commiting
5. Never merge your own pull request (unless its something obvious)

# To Do List

### 1. Set up Tailscale on all devices
#### 1.1. Instructions will be here...

### 2. Add the world with the blocks to the repo
#### 2.1. I (Denis) will do it over the weekend

### 3. Design two rails for the grabber (Neil)
#### 3.1. Create a CAD model
#### 3.2. Add it to the TurtleBot3 model in Gazebo

### 4. Start writing code
#### 4.1. Block recognition (Duncan)
#### 4.2. Arena discovery (Duncan)

### 5. Design the camera mount (Duncan)
#### 5.1. Create a CAD model
#### 5.2. Add it to the TurtleBot3 model in Gazebo


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
ros2 launch turtlebot3_task turtlebot3_world.py red_marker:=<AruCo marker> blue_marker:=<AruCo marker>
```
Available AruCo markers are 0 (W), 7 (N), 23 (E), 42 (S).  

## Checking Blue and Red Markers

After launching the world, if you run 
```
ros2 param get /competition_logic (blue/red_goal_id)
```
it will tell you which marker has been assigned that colour 

## Note:
All executable code goes to the turtlebot3_task folder, any worlds into worlds folder, models into models, etc. (look in the turtlebot3_gazebo folder for an example).
