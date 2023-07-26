# KubeROS Basic ROS2 Examples 

This repos contains some basic ROS2 examples to show you how to 
 - develop a ROS 2 application in a .devContainer
 - containerise software modules using docker 
 - create a KubeROS deployment manifest
 - deploy to the KubeROS platform



## Examples
 - Hello world 
 - Add ints & Fibonacci
 - Turtlesim 


## Others
 
Allow the root user on the local machine to connect to X server 
```
xhost + local:root
```

For using in computer with Nvidia GPU: 
add following variables in `.devcontainer/docker-compose.yml`
```
    environment:
      DISPLAY: $DISPLAY
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
    runtime: nvidia
```


### Todos: 

- Warning while building with colcon, check it later
```
--- stderr: examples_rclpy_minimal_publisher
/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
```


