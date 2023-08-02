# IoT4Ag - File Description
## Jackal_all.zip:
- Contains /jackal, /jackal_desktop, and /jackal_simulator
- Location: inside catkin_ws/src (any catkin workspace)


### Important Folders:


**cd /jackal_simulator/jackal_gazebo/**
  - /jackal_simulator/jackal_gazebo/launch/my.launch
    - Launch file for the red ball world (/jackal_simulator/jackal_gazebo/worlds/red3.world)
    - To launch:``` roslaunch jackal_gazebo my.launch ```
    - Spawns jackal on line 30 (<include file="$(find jackal_gazebo)/launch/spawn_jackal.launch">)
- /jackal_simulator/jackal_gazebo/launch/spawn_jackal.launch
  - Includes jackal_description/launch/description.launch on line 13

**cd /jackal**
- Contains /jackal_description/urdf and /jackal_navigation (and other files but these two seem more relevant)
- Description defines the jackal robot and its accessories

- jackal/Jackal_description
  
  - /jackal/jackal_description/urdf/realsense.urdf.xacro
    - Contains the realsense camera
  - /jackal/jackal_description/urdf/accessories.urdf.xacro
    - links accessories for the jackal robot
  - /jackal/jackal_description/urdf/jackal.urdf.xacro
    - Jackal defined
 
    
**cd /jackal_desktop**
Includes the rviz


## Segment and lawnmower.zip:

Location: inside /ViBES-sensor_segment_colleen/ViBES-sensor_segment/src


## To Run the gazebo world and robot simulation
1. ```roslaunch jackal_gazebo my.launch```
2. ```cd ~/Downloads/ViBES-sensor_segment_colleen/ViBES-sensor_segment/src/sensor_segment/scripts```
3. ```python3 segment_online.py```
4. ```python3 waypoints2.py```

