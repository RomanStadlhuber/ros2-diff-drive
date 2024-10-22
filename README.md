# ros2-diff-drive

## Important Remarks

When running the simulation launch file, do not forget to **press play** :)

## Quick Start

### Build

Inside the `robot_ws/` root, run
```bash
$ colcon build
$ . ./install/setup.bash
```

### Launch

Depending on whether you want to simulate the robot in gazebo or actually run the real robot, you may want to use one of the two main provided launch files.

#### Launch the Simulation Packages

Inside the `robot_ws/` root, run
```bash
$ ros2 launch ./launch/simulation.launch.py
```

<br/>
This will then start ignition gazebo as well as the required ros nodes.

#### Launch the Robot

To be announced.




### Control the Robot

Right now the only way to control the robot is to use the publish functionality of the ros2 cli.
After installing all the packages, launch the simulation or minimal setup and then run
```shell
    # NOTE: insert the desired values for wl and wr - corresponding to the angular velocities of the left and right wheels
    $ ros2 topic pub --once /diff_drive/angular_vel_in  angular_vel/msg/DiffDriveOmega "{wl: 0.0, wr:0.0}"

    # NOTE: to view angular and planar velocity or position and rotation (i.e. pose), run
    $ ros2 topic echo /diff_drive/cmd_vel # for linear and angular velocity
    $ ros2 topic echo /diff_drive/pose # for position on the xy-plane and rotation on z-axis

    # for an exhaustive list of topics includeing types, run
    $ ros2 topic list -t
```

## Packages

### angular_vel

Message definition for angular velocity of the differential drive's wheels.

#### DiffDriveOmega

See **msg > DiffDriveOmega.msg** for message definition
</br>
- **wl** - angular velocity (omega) of left wheel
- **wr** - angular velocity (omega) of right wheel

##### Build the Package

To build:
</br>
`robot_ws:$ colcon build --package-select angular_vel`
</br>
To install:
</br>
`robot_ws:$ . install/setup.bash`
</br>
To test message definition:
<br>
`robot_ws:$ ros2 interface show angular_vel/msg/DiffDriveOmega`
</br>
Outputs:
```
float64 wl # omega left wheel
float64 wr # omega right wheel
```


### wheel_control

#### wheel_driver (node)

Sends commands to the wheels and receives information about current angular velocity of each wheel.

### localization

#### odometry (node)

Uses `wheel_driver`s information about angular velocity of both wheels to calculate angular and planar velocity of the robot.

#### localize (node)

Performs planar localization based on `odometry`s current angular and planar velocity information about the robot.

## Misc

### Visual Studio Code Integration

Within your C/C++ configuration, be sure to add the include path of your ROS2 distro (I am using foxy) within the include-directories of the `c_cpp_properties.json` file like so:

```
{
    "configurations": [
        {
            
            ...

            "includePath": [
                "/usr/include/**",
                // list your ROS2 distros include dir here!
                "/opt/ros/foxy/include/**",
                // include custom message definitions
                "${workspaceFolder}/robot_ws/install/angular_vel/include/**"
            ],
            
            ...

            // see the standard I am using
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17"
        }
    ],
    ...
}
```