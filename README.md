# ros2-diff-drive

To be announced

## Packages

### wheel_control

### wheel_driver (node)

Sends commands to the wheels and receives information about current angular velocity of each wheel.

## simulation_control

### gazebo_bridge (node)

Sends information from `odometry` about current angular velocity and planar velocity to the `cmd_vel`. Ignition Gazebo's diff-drive plugin will listen to that topic and move the robot accordingly.

## localization

### odometry (node)

Uses `wheel_driver`s information about angular velocity of both wheels to calculate angular and planar velocity of the robot.

### localize (node)

Performs planar localization based on `odometry`s current angular and planar velocity information about the robot.


### Visual Studio Code Integration

Within your C/C++ configuration, be sure to add the include path of your ROS2 distro (I am using foxy) within the include-directories of the `c_cpp_properties.json` file like so:

```json
{
    "configurations": [
        {
            
            ...

            "includePath": [
                "/usr/include/**",
                // list your ROS2 distros include dir here!
                "/opt/ros/foxy/include/**"
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