# Ur5e General Use
This repo includes general information about the use of Ur5e with ros2. Expamples for running joint positions, inverse kinematics, trajectory planning and isaac sim simulations

## Using fake hardware 
For motion test with no real hardware use:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 use_fake_hardware:=true 
```
Any IP adress can be used, only to follow string convention syntax
