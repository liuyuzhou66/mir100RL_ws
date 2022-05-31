# new_mir_gazebo

## gazebo:
```
roslaunch new_mir_gazebo mir_mall_world.launch

rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
```

## Set the current pose of MiR robot:
```
roslaunch mir_navigation amcl.launch initial_pose_x:=2.0 initial_pose_y:=2.0
```
or alternatively: 
```
roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0
```

## Multi goals navigation:
### Navigation:
```
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find new_mir_gazebo)/maps/mallmap.yaml \
    with_virtual_walls:=false
```
### Launch moving obstcales(shoppers, employees):
```
roslaunch new_mir_gazebo moving_obstcales.launch
```
### Launch Rviz:
```
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```
### Publish navigation goals and parameters:
```
roslaunch new_mir_gazebo movebase_seq.launch
```
