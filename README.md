# Ur5e General Use
This repo includes general information about the use of Ur5e with ros2. Expamples for running joint positions, inverse kinematics, trajectory planning and isaac sim simulations

## Using fake hardware 
For motion test with no real hardware use:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 use_fake_hardware:=true 
```
Any IP adress can be used, only to follow string convention syntax

## Isaac Sim 
After a success instalation considering CUDA and NVIDIA drivers, verify instalation and run 

```bash
local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh
```
This is an Isaac Sim [extension](https://github.com/ferestradaa/ur5e/tree/main/ik_extension1) for IK solving using RMPFlow solver. It automatically publishes new angular joint position for UR5e robot. Ready for ROS2 implementations

