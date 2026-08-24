# Argus Sim

Argus World Sim provides a gazebo based simulation environment for the Argus Cybernetics Stack.

## Local Workflow

Reformat this repos source files:
```
cd ~/Documents/argus_ws
source install/setup.bash
ament_uncrustify --reformat src/argus_sim/src src/argus_sim/include
```

Verify reformatting was successful:
```
ament_uncrustify src/argus_sim/src src/argus_sim/include
```
