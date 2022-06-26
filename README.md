# mir100RL_ws

## Greedy Algorithm
```
clear && roslaunch mall_robo_gym mir100_greedy_1.launch
```
### Plot the testing result
#### Without obstacles (10 episodes)
![Greedy_Rewards_and_OverallTime](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/Greedy_OverallTime.png)
#### With obstacles (10 episodes)


## Reinforcement learning
### Start the training
```
clear && roslaunch mall_robo_gym mir100_rl_1.launch
```
### Test the training outcome
```
clear && roslaunch mall_robo_gym mir100_rl_test.launch
```
## Path to change the time of controller_patience and planner_patience
```
cd ~/mir100RL_ws/src/mir_robot/mir_navigation/config/move_base_common_params.yaml
```
- controller_patience: 15.0    # if the controller failed, clear obstacles and retry; after 15.0 s, abort and replan
- planner_patience: 5.0        # if the first planning attempt failed, abort planning retries after 5.0 s

### Map of 3d environment (1435 m2)
![Map](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/src/new_mir_gazebo/maps/mallmap.png)
![Mocel](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/mall.png)
- Length: 41 m
- Width: 35 m
### Legend for waypoints in gazebo
![Legend](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/Legend.png)
- Blue sphere: those waypoints that have not been visited yet
- Green sphere: those waypoints that have been visited
- Red sphere: the waypoint that the robot is currently heading to

### Plot the training result
#### Without obstacles (800 episodes)
![RL_Rewards_and_OverallTime](https://github.com/liuyuzhou66/mir100RL_ws/blob/run_2022-06-24/Results_Plot/RL_Rewards_and_OverallTime.png)
- The minimum time for the [mir100_rl_1] to complete the task is  seconds in episode <> with reward ()!
- Overall time of training: 17.65 h = 1059 min
- Training time per episode: 4.27 min
#### With obstacles (800 episodes)

### Plot the Q table
#### Without obstacles (800 episodes)
![RL_Q_table](https://github.com/liuyuzhou66/mir100RL_ws/blob/run_2022-06-24/Results_Plot/RL_Q_table.png)
#### With obstacles (800 episodes)

### Plot the testing result
#### Without obstacles (10 episodes)
![RL_Test_no_obs](https://github.com/liuyuzhou66/mir100RL_ws/blob/master/Results_Plot/RL_OverallTime.png)
#### With obstacles (10 episodes)
