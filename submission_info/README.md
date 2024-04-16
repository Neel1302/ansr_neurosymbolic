# Overview
This repo is intended to be used as part of evaluation 1 for the DARPA ANSR project. *NOTE:* We are a manuever thread performer for evaluation 1.


# Requirements
This repo requires a Ubuntu 22.04 Linux environment capable of running the ADK Airsim docker images system with an nvidia GPU with Cuda and docker.

The minimum required VM size is: NC12s_v3 with a 12 core Intel Xeon E5-2690 v4 CPU with 224 GB RAM and dual Nvidia V100s with a total VRAM of 32 GB. *Note*:  we actually run our docker container on a machine with a 16 core 13th Gen Intel® Core™ i9-13900K CPU with 32 GB RAM and a RTX4090 with VRAM of 24GB.

We tested our code with ADK with ID `sha256:79732673c0f72bd3afca55b6567c662c56f8530fcc990bab8c2165999fd56d91` and we expect our code to be evaluated using this ADK image.

# Testing

Our image can be tested by using the deploy docker compose file. Our docker container contains all the neccessary mounted volumes and only relies on 
the packages built into the container.

  ```
  docker compose -f docker-compose.deploy.yml up
  ```

Our code is automatically executed when the docker container is deployed and we achieve this using the `ros_node.sh` script with runs the following commands in the `deployment` container:

```
source "/home/performer/dev_ws/install/setup.bash"
python3 "/home/performer/dev_ws/src/verifiable-compositional-rl/src/ansr_eval1/waypoint_publisher.py"
```

Since we are a manuever thread performer for evaluation 1, you may echo the `adk_node/input/waypoints` topic to view the waypoints we publish.
    
  ```
  docker compose exec deployment bash
  (in the container) source install/setup.bash
  (in the container) ros2 topic echo adk_node/input/waypoints
  ```

# High-level overview of our code (inputs/outputs)

Our main algorithm is housed inside the file named `waypoint_publisher.py` at `/home/performer/dev_ws/src/verifiable-compositional-rl/src/ansr_eval1/waypoint_publisher.py`.

We have an rclpy node instance named `mission_exec` and where we house a ros node named `waypoint_publisher`.

We subscribe to the following topics:

`adk_node/ground_truth/perception`
`adk_node/SimpleFlight/odom_local_ned`

and we publish the following topics:

`adk_node/input/waypoints`
`adk_node/input/terminate`

We have commented our code for further explanation. Our code cannot be run without ROS at present.