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
1. Load from isaac assets both kinova gen3 usd (body) and gripper usd with name ending in edit
2. For every usd, make sure youre using a flattened and non instanceable version (export as flatenned and remove instances)
3. For safety change links names in case theyre repeated even if differrent usd files
4. Create a father prim for the complete robot and set it as articulation root
5. Inside this father, place both the body and gripper, make sure they dont use an articulated root but they do rigid bodies
6. Join the bodys bracelet link and the base link of the gripper with a fixed joint. MAke sure to manually adjust the local pose.
7. To verufy it works, create 2 articulation controllers and move the joints (finger_joint and any robots joint) in order to check you can access directly the articulation root.
8. Set default prim the parent.
9. Delete the action graphs used for testing when exporting to a clean scene

## Build USD for any robot

1. Imporrtar el cuerpo del robot, borrar el prim del endffector porque se agregara el gripper mas adelante. Guardar como flatenned y asegurarse de que solo su prim padre sea root articulation. Verifica que al dar play se pueda mover sin problemas

The following scripts help the user to very if their usd works for rmplow solver. Try each of them on isaac sim script editor


1. Verify if the usd has an Articulation
```python
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import add_reference_to_stage

robot_path="/gen37_instanceable"
robot = Articulation(robot_path)

robot.initialize()  
robot.post_reset()

print("DOF:", robot.num_dof)
```


