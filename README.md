# KubeROS Basic ROS2 Examples 

This repos contains some basic ROS2 examples to demonstrate the workflow with KubeROS: 
 - development in containers (with .devContainer)
 - packaging each software module in a standalone container (microservices oriented)
 - debugging with multiple containers in local environment
 - create `KubeROS Deployment Manifest` for scalable deployment in multi-robots, optionally, with computing resources from edge or cloud. 
   - `RosModule`: smallest unit in KubeROS, contains the container image addresses, command to execute, launch parameters, requirements, `rosParameters` to dynamically load custom parameters from `rosParamMap`. This definition should be maintained by the developers with clearly declared parameters for the users.
   - `RosParamMap`: interface to provide the parameters according to the required `rosParameters` from `rosModule`. Support two types: `key-value` and `yaml`
   - `staticFile`: interface to attach large files such as ML-weights to the container.
 - interact with KubeROS API server to deploy and orchestrate the containerized software modules


## Hello world

Simple example with two nodes: 
 - Publisher with custom parameter from launch and yaml file.
 - Subscriber: get the message and print it in the terminal

Containers: `metagoto/ros2-basic-examples_hello_world:humble-v1.0`
Since this example is simple, we package all nodes in a container based on `ros-core' to achieve a small image size. In the deployment, two `ROS nodes' are started in separate container instances.

[Deployment manifest](deployments/hello_world/hello_world.deployment.yaml) contains the following important information for deployment, for more details see the YAML file:
```yaml
metadata:
  targetFleet: xx 
  targetRobots: ['simbot-1'] # (Optional) If not specified, the application will be deployed to all robots in this fleet.
rosModules:
  - name: hello-world-talker
    image: <image-address>
    requirements:
    launchParameters: # launch parameters used in ros2 launch <parameter x>
    rosParameters: # Define from which rosParamMap the parameters should be taken.

rosParamMap:
  - name: helloworld-launch-parameters # Name must be equal to the valueFrom in rosParameters.
    type: key-value # key-value or yaml
  - name: publisher-param.yaml
    type: yaml
    path: config/publisher_param.yaml # Path to the YAML file on your local computer.
```



## Testing in local env with docker
Create a new network interface ([Bridge network](https://docs.docker.com/network/network-tutorial-standalone/#use-user-defined-bridge-networks))
```bash
docker network create --driver bridge ros-net
```
Then you can start all containers with the specified network interface `ros-net`. You can also use the `host' network setup. However, we recommend using a dedicated network bridge for controllable network traffic.



## Working with DevContainer
 
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

