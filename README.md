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

## Resources

https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_motion_generation_rmpflow.html

https://docs.omniverse.nvidia.com/isaacsim/latest/concepts/motion_generation/rmpflow.html#isaac-sim-motion-generation-rmpflow

## Give a check to this intreractive filtering demo

https://gery.casiez.net/1euro/InteractiveDemo/

# Notes

1. Para nodos de deteccion y conversion a 3D, el timestamp debe ser medido aun cuando se trabaja con frames (tiempo discreto) de forma que realemnte pueda medirse el tiempo de ejecucion entre cada deteccion.
Por ello, el tiempo es parte del mensje que se publcia y recibe.
Despues, al momento de aplicar filtros, con esto se asegura que cada componente esta siendo evaluada sobre el mismo instante de tiempo, evitando inconsistencias cuando
se visualizan los resultados


## RMPFLow setup for new robots
1. Use Lula Robot Description Editor to generate a YAML file used for lula based algorithms suchas as RMPFLOW.

## Build USD for any robot

1. Imporrtar el cuerpo del robot, borrar el prim del endffector porque se agregara el gripper mas adelante. Guardar como flatenned y asegurarse de que solo su prim padre sea root articulation. Verifica que al dar play se pueda mover sin problemas
2. 



