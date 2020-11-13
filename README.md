# ros2-diff-drive

To be announced


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