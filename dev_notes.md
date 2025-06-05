# Development Notes
- Running on WSL2, Ubuntu-22.40 and Docker
- On WSL2 Terminal, rqt works as intended but inside container no go
- Trying to install VcXsrv and set it up first
- set .wslconfig to have mirrored network
- dyn_obstacles.launch.py spawns multiple obstacles, can refer to it to see how we might spawn more than 1 agent. Note robot_state_publisher and spawn_entity
- if installing new stuff try to put at the end of docker file for the cache to speed up build
- for base dynus launch, need to source /usr/share/gazebo/setup.bash
- as of this line, this repo/docker combination works out of the box. after starting the container with `make run-sim`, run the single agent in forest via `bash ./dynus.sh`
- to see ros2 topics, `export ROS_DOMAIN_ID = 7`, making all panes/terminals the same
- fake_sim is the node that moves the drone! updated state is published every 10ms. how the state is updated is very simplistic in fake_sim, here the new state and velocity is taken to be that of the set point and orientation is a function of setpoint acceleration.
- timesynced node/subscriber that listens on state, goal. but the next state and current state exist on the same topic! next state is computed from goal i.e. next state is a function of goal
- `ros2 run dynus flight_data_collector_single.py --ros-args -r __ns:=/NX01`
- for now, just pull dynus in container and rebuild before deving directly in container. abit cursed but ya.
- need to think about reference control input. 
- attaching volume to container with write permissions to collect data
- as of this line, can run easy_forest with either `dynus.sh` or `tmuxp`
- multiagent not running atm, likely due to some memory thing, should try next work day. the fix `--ipc=host` apparently is not secure but yet to try. alternative is `-v /dev/shm:/dev/shm`

#### 27/5/2025 Tuesday
- not memory issue. process dies (gzserver) when a second agent is launched and the realsense_camera plugin is attached to another model/plugin, i.e., being set up for the second agent.
- added some debug statements in `RealSensePlugin`, currently cameras not being initialized. Yup, even though depth cam name is unique (properly namespaced), still cannot initialize. note that the failure is independent of which agent gets spun up first. NX02 before NX01 means NX02 OK NX01 NOK and vice versa. a deeper problem that i don't think is worth the time to get into as of right now, especially considering the simulation's complexity.
- to fork `realsense_ros` because the original crashes when the depth camera cannot be initialized. the current changes that i've made is such that we just proceed without a depth camera, so only have lidar. checking to see if we have that at least, yes we do.
- can run multiagent sim setup, but takes damn long to spin up. ie NX01 spins up quick, but subsequent agents take expoenentially longer to spin up. Subsequent agents also do not have depth camera BUT have lidar and thus can still work. 
- need to time alot of stuff together because what seemed to happen was that even though both goals published simultaneously, because NX02's lidar simulation was not set up yet, nothing happened for it. Only a decent while after NX01 reached its goal, then NX02 moved. so only publish goal after both's lidar sim is set up.
- no proper means of visualization through rviz or gazebo. gazebo gui does not run by default but has been crashing on run, even in single sim (to check again) and thus is kinda worthless. possible to do through rviz, but means need to create/expand on existing rviz file. possible to do so programmatically since same format? i.e. require the same information to be observed.
- camera feed is only live for the first agent. even so, latency is horrible so you can't see what's happening anyway. 
- actual performance: not sure the reason why, but sometimes the gurobi solver fails for a long while and the agent just remains stuck. but is somehow able to resume movement after a while. is it moving after a successful recompute or did it get stuck because of resources or just doing whatever. hard to tell in rviz. 
- successfully created means of flight data collection and visualization. static visualization of flight path using matplotlib but real-time is difficult (and worringyly impossible honestly)
- the above has also been tried with 3 agents, but the setup time increases exponentially. have yet to try yet with obstacle_tracker as true, which ironically is the selling point of dynus, so to try that.
- next steps: collect flight data (simultaneous, not one after the other) for 2 agents, visualize and compare with obstacle tracker on too. can also try different maps? think of funky route to stress the agents. essentially data collection/visualization phase for now? 
- opportunity to also rewrite `macbf` in pytorch, since we cannot use the pre-trained TF version anyway (different state spaces across our cases). should have a `Scene` helper class to make life easier. 

#### 30/5/2025 Friday
- Made `Scene` helper class. 
- Ported `NetworkAction` to `PyTorch`
- Ported `NetworkCBF` to `PyTorch`
- Ported the various loss functions to `PyTorch`
- To further develop training loop and evaluation loop
- Next steps: Planning and collection of flight data. Not sure how to do this efficiently especially when I cannot know how the different maps look like, gazebo gui cannot run :(

#### 2/6/2025 Monday
- Got cucked by ROS2 humble docker images. Their keyring expired. Looked up Github issues and thankfully found a workaround.
- Ran 2, separate, single agent runs, same goal and captured their flight paths. End points are same but flight trajectories are vastly different. Planner is non-deterministic in a static environment.
- Setting up the obstacle tracker node. Updated the QoS policy to make it work. It should have been working already, not sure why I needed to mess with source code. Will try to see activating obstacle tracker in single agent environment. 
- Capture flight data of two agents with obstacle tracking active (one scene) and try to train and evaluate on it first. For now, evaluate means to just run the trained control policy over the collected flight path as per the original MACBF implementation and visualize the new flight path and how it differs. A second stage of evaluation is to create a new `fake_sim` i.e. controller that uses this learned control policy and collect the same flight data on the same goal. For this to be "correct"/"good" would require a lot of training data, given that the flight paths generated by DYNUS are seemingly non-deterministic and thus we would require a lot of training (i think) to achieve model generalizability. 
- Maybe we can try to see how/why gazebo gui fails for now. I think if we can get that up it will be very helpful for visualization. 
- dynamic obstacles can spawn and be seen in gazebo. 
- gazebo visuals now fixed. now we can see flight motion in gazebo! 
- collected flight plans. usable ones are easy_forest 0 to 2, five_randoms and three_random_small
- dynus actually fails sometimes and the uav just gets stuck and refuses to move, i think it might be the environment. sometimes it fixes itself sometimes it doesnt.
- for tomorrow, we work on the training and eval pipeline first and try to go through these pieces of data first, just to see that it works. after that if time permits we collect data on office and exploration. comments seem to suggest that exploration starts out the box, but we might need to make a publish call or something.
- there are two goal sender calls we can consider in the dynus readme. forest3 and high res forest. also see how we can modify docker multiagent sim to collect this. i think data collection might just end there

#### 3/6/2025 Tuesday
- Implemented training script in `macbf-torch`
- Key learning points: Without `detach`, `current_state`'s history continues to grow (i.e. it is a function of/linked to the `current_state`s that came before it) and thus gradient computation has to traverse through this entire history. `backward` only clears the history between `loss` and `current_state`, not `current_state`'s growing history!
- Also noticed that in original TF implementation, gradients were accumulated and averaged. Made the same change here.
- Made eval script too. currently only supports one scene at a time for sanity. if okay then we make it take more at once.
- The default `dynus.yaml` has `terminal_goal` and `use_frontiers` which is not supposed to be true. For ease of use, we can use the `easy_forest_param.yaml` to run in the `forest3` environment
- 1200% CPU usage according to docker for sim when collecting data for 3 agents
- Collected some data in `forest3`
- Training seems to be correct but slow, going at ~130it/s right now. Before the `detach` fix, was ~3it/s, something to be happy about i guess. 
- Across both implementations (PyTorch/TF), they are training scene by scene i.e. flight path by flight path. Is it possible for us to batch and train across scenes in parallel? Right now, a scene is `TxNxs`. Adding a batch dimension would give us `BxTxNxS` where `B` is batch, `T` is timesteps, `N` is number of agents and `S` is state. 
- Indexing into a particular `T` yields `BxNxS`. Semantically that means state per scene at that particular time. Issue of padding all scenes to be of the same length/timestep duration. 
- Even if we can pad, I think that's the least of our problems. What does it semantically then make sense to put `BxNxS` through the network. Mask computations? In theory, I think might be able to do? But is it worth? What are the drawbacks? When we pad, we actually pad the last entry to simulate hovering when goal reached. Issue with this right now is that the model might overfit to controller adherence and lesser CBF satisfaction because we don't have sufficiently dangerous states for the model to learn how to deal with that
- Which brings me to my next observation as well, currently danger losses are quite close to zero because my collected scenes at the start when I was just trying out unfortunately didnt have the agents close to another, so they were actually in safe states most of the time.
- But to combat this, i've written a `random_waypoints_sender` which publishes goals within a defined cube/cuboid, to hopefully increase the chance of danger states. sometimes run into the problem of dynus freezing though and the drones stop moving. some luck involved i guess. 
- tomorrow should be the last day of data collection, any more and we'll never train finish haha. perhaps quality over quantity. today's `forest3` scenes should be usable. maybe just 3 more boxed movements and we stop there. 

#### 4/6/2025 Wednesday
- eval script works. The plotted graph from evaluation closely resembles that of the original of which it was trained on so that's good because its expected i.e. follow original control method if safe. Upon closer inspection of the generated CSVs (specifically NX03 flight path of `three_agents_obs_track_easy_forest_0`), we observe that the path deviates at the start, before "catching up" at around state 20. From there on out, it closely follows that of the training path, with very minute deviations. I suppose the deviations at the start were too small to be captured by `matplotlib`, and hence is why the graphs look the same across both evaluation and training. 
- I suppose it's also expected that the path following is not precisely exact, given that it is a learned model with some randomness injected to it.
- Improve eval script to process multiple scene directories
- Collected few more flight data in `forest3`, will stop here and just train
- to think about deploying trained model in sim. currently each agent has its own `fake-sim` which is the "state-overwriter". create a centralized version? centralized node that subscribes to 3 goals, each on an exclusive callback to update the state matrix. current and reference state matrix can be put into control policy, multiply by dt to get next state matrix publish this current state matrix. downside is that movement is simulated agent by agent because of exclusive callback but no issue with race conditions in the sense that we ever only update one agent at a time. 
- could also do an approxtime sync for goals. actually seems fine in theory.
- 3 individual `fake-sim`s like the current dynus setup? 3 instances of the model. instead of subbing to goals, can sub to states of non-self agents to update state matrix. sub to goal of self.
- okay i think it make more semantic sense for us to try to align with the *decentralized* part of MACBF, which claims that each agent has its own controller rather than a central controller.
- in the eval script of the original MACBF however, reference states are simultaneous. by that i mean, it appears they are simulating a central controller which has knowledge of the current states, by extension the "observations" of neighboring agents and goal states of all drones and the policy simultaneously outputs a control input that modifies the states of all agents simulataneously in one timestep. seems like a centralized controller to me. 
- $\pi_i(s_i, o_i, s_{\text{ref}, i})$. let's try to make it like that. so node subbed to self goal, non-self states as observations to construct the state matrix. reference state matrix is merely the original but with the self state vector being updated by the goal. non-self state callbacks to just alter the current state matrix entries of non-self. self goal callback to construct reference state matrix, go through model, extract self control input and apply to current state. update current state matrix and publish updated current state. non-self updates need not be synchronised since the update can happen quite fast. 
- the next challenge that comes in evaluating the value added by appending MACBF onto DYNUS. difficult for us to isolate the fact that any "safe" behaviour is because of MACBF alone, independent of DYNUS, but I suppose that is also the selling point of this setup. two layered safety "guaranteed" at the planning/trajectory level and at the movement/control level. 
- a shitty idea might be to compare safety ratios between the usual controller and our new controller. however, because the paths that dynus plan are not deterministic, it might not be a good gauge, but better than nothing i suppose.

#### 5/6/2025 Tuesday
- Created `mac-sim` node that does what we sought to do as described above. Haven't test yet because training is still ongoing and I haven't figured out how to properly extract the weights to be put into the `share` folder during `colcon build` but that's a minor point.
- Should have made a better loss logging system, maybe graphs or something but abit too late to terminate training atp. Some screenshots do actually reveal learning where the danger/safety losses are going down, so a good sign.
- Also ported over the online refinement policy over to `core` and tested in `eval`. It's only ever invoked properly when the agent is in unsafe state, if safe, no refinement takes place which is fine. 
- made some `detach` fixes so that the refinement gradient computation can take place. learning more about pytorch's computation graphs. 
- chat with mentor: means of visualization can be gazebo, logging of safety ratios, change the frequency of publishing to reduce training size, create smaller training boxes. agreed that batched training might not make the most sense. 
- running evaluation out of the box without loading state dicts shows us that the path following is quite good already, i think the original controller is already doing most of the heavy lifting. confirmed when we evaluate by setting control input as pure model output, path following fails completely. to be expected in a way because the model is only supposed to alter the output from the original controller not completely replace it.
- todos: inspect the config files for dynus to see how we can reduce the frequency of publishing. let's try 0.1s from the current 0.01s. also to create smaller boxes to encourage more dangerous states. try 10x10x1. MACBF uses a 10x10 square (all fly on same plane). 