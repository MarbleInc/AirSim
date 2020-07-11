# Record an Airsim Simulation Sequence For Perception Testing

Recording a simulation sequence involves recording both a sensor bagfile and a directory of ground truth data in json form.



## Recording ground truth data

### Launch Unreal Engine
* Start the UnrealEngine with the following command

`
./Engine/Binaries/Linux/UE4Editor
`


* Open the project located in `Airsim/Environments/Empty/Empty.uproject`



* Migate 
`Airsim/ros/src/mbot_ros_pkgs/data/settings.json` to `~/Documents/Airsim/settings.json` where Airsim will expect it.



* Hit the play button on the Unreal Environment


### Launch Airsim wrapper
`
roslaunch mbot_ros_pkgs mbot_sim.launch
`

### Begin Recording a sensor bagfile
`
rosbag record -a -O simulation.bag -x "/tracked_objects_simulated"
`

### Record a ground truth sequence
###### Begin Recording
`
rostopic pub /record_gt_data std_msgs/Bool "data: true"
`

###### End Recording
`
rostopic pub /record_gt_data std_msgs/Bool "data: false"  
`

### End Recording a sensor bagfile
`
Ctrl-C on rosbag record
`