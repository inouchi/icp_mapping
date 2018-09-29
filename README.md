# ssh_icp_mapping
This package is for generating a 3D model using a Kinect v1 or v2.  
We use ICP algorithm to align distance data obtained from a variety of angles.  
Note: You must build an enviroment for using Kinect v1 or v2.


# Usage
As shown in the following movie, a measurement object is placed on a turntable.  
You rotat the turntable to get distance data of the measurement object from a variety of angles.  
Note: The turntable and Kinect must be installed horizontally on the ground.
  
The rosnode for ssh_icp_mapping is activated with the following command.
```bash
$ roslaunch ssh_icp_mapping icp_mapper.launch
```
If you want to generate an initial model, use the following rosservice.  
By setting isFirstBoost to true, ICP processing is not performed, and a filtered measurement data is registered as an initial model.  
```bash
$ rosservice call /icp_mapper_node/generate_model "isFirstBoot: true degree: 0.0"
```
You use rosservice as follows (e.g. Rotation angle is 30 degrees) when you want to integrate measurement data using ICP.  
The input rotation angle is an absolute angle with reference to an angle at the time of generating an initial model.
```bash
$ rosservice call /icp_mapper_node/generate_model "isFirstBoot: false degree: 30.0"
```
  
You can return to the state of the previous model.  
Please use the function if integration by ICP fails.  
Note: It can not return to the state of two or more previous model.
```bash
$ rosservice call /icp_mapper_node/restore_previous_model
```
  
![experiment](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/ssh_icp_mapping/raw/media/experiment.gif)

# Parameters
All the parameters are listed in launch/icp_mapper.launch as ros params.  
You need to change the parameters as necessary.


# Generated 3D Model
![3d_model](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/inouchi/ssh_icp_mapping/raw/media/3d_model.gif)
