# Extended Minimal Publisher

This package is based on the basic ros2 example with the following changes:

 - Launch parameters `init_topic`: as one of the launch parameters that will be pulished in the first `10` secs and can be set directly in `RosParamMap`.
 - Launch parameters `params_file`: yaml file path containing `ros__parameters`to be loaded into parameter server.
    - `topic`: 'world from container'
    - `talker`: 'humble'
    - `content`: 'launched by kuberos with custom parameters'
