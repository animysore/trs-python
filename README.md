# TRS Python

Python port of [TRS: Teaching/Learning Robotics with a Simulator](https://github.com/ULgRobotics/trs) by Renaud Detry.

TRS is an open-source recipe that provides a template for starting a robotics project using VREP/CoppeliaSim using Python bindings. It is a port of the original recipe for MATLAB.  

Please check the [TRS website](http://ulgrobotics.github.io/trs/) for more information about what is provided.

**Note** - This is a work in progress.

## Setup

1. Download [VREP/CoppeliaSim](coppeliarobotics.com/).

2. Edit `config.ini` - Set `VREP` to the path to where CoppeliaSim is installed, and `VREP_LIBRARY` the path to the remoteAPI file. This will be `remoteApi.dll`, `remoteApi.dylib` or `remoteApi.so` depending on your plaform. Check the [CoppeliaSim documentation](https://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm) on how to locate the path to the file for your platform.

   The following is a sample `config.ini` for a Linux environment. This will change based on where you download the simulator!

   ```ini
   VREP: /home/aniruddha/CoppeliaSim/
   VREP_LIBRARY: /home/aniruddha/CoppeliaSim/programming/remoteApiBindings/lib/lib/Ubuntu18_04/
   ```

3. Open the simulator and load the default scene - `scenes/house.ttt`.
4. Run any of the examples in the `focused` directory.

    ```bash
       cd focused
       python youbot_arm.py
    ```
 