# mir100RL_ws

## Start the training
```
clear && roslaunch mall_robo_gym mir100_rl_1.launch
```
## Path to change the time of controller_patience and planner_patience
```
cd ~/mir100RL_ws/src/mir_robot/mir_navigation/config/move_base_common_params.yaml
```
- controller_patience: 15.0    # if the controller failed, clear obstacles and retry; after 15.0 s, abort and replan
- planner_patience: 5.0        # if the first planning attempt failed, abort planning retries after 5.0 s

### Legend for waypoints in gazebo
![Legend](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/Legend.png)
- Blue sphere: those waypoints that have not been visited yet
- Green sphere: those waypoints that have been visited
- Red sphere: the waypoint that the robot is currently heading to

## Plot the result
![Rewards_and_OverallTime](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/Rewards_and_OverallTime.png)

## Plot the Q table
![Q_table](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/Q_table.png)
